#include <iostream>
#include <csignal>
#include <string>
#include <mutex>

#include <opencv2/opencv.hpp>
#include "opencv2/gpu/gpu.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <dji/guidance.h>
#include <dji/utils.h>
#include "guidance_manager.hpp"
#include "guidance_configuration.hpp"

#define CAM_LEFT true
#define CAM_RIGHT false
#define GRAVITY 9.80665

std::mutex g_guidance_mutex;
int guidance_data_rcvd_cb(int event, int data_len, char *content);

#define RETURN_IF_ERR(err_code)                                                \
  {                                                                            \
    if (err_code) {                                                            \
      release_transfer();                                                      \
      std::cout << "Error: " << (e_sdk_err_code)err_code << ' '                \
                << dji::err_code_str(err_code) << " at " << __LINE__           \
                << "," << __FILE__ << std::endl;                               \
      return err_code;                                                         \
    }                                                                          \
  }

e_sdk_err_code GuidanceManager::init(ros::NodeHandle pnh) {
  pnh_ = pnh;
  config.applyFromNodeHandle(pnh_);
  it_ = new image_transport::ImageTransport(pnh_);

  // Init message buffers
  image_left_.image.create(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
  image_right_.image.create(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
  image_depth_.image.create(IMG_HEIGHT, IMG_WIDTH, CV_32FC1);
  mat_depth16_.create(IMG_HEIGHT, IMG_WIDTH, CV_16SC1);
  image_cv_disparity16_.image.create(IMG_HEIGHT, IMG_WIDTH, CV_16SC1);
  image_cv_disparity32_.image.create(IMG_HEIGHT, IMG_WIDTH, CV_32FC1);
  // Invalidate angular velocity since it's not provided, mark the covariances
  // as unknown
  imu_msg_.angular_velocity.x = imu_msg_.angular_velocity.y =
      imu_msg_.angular_velocity.z = 0;
  imu_msg_.angular_velocity_covariance = {-1};
  imu_msg_.linear_acceleration_covariance =
      imu_msg_.orientation_covariance = {0};
  // Set angular twist to 0
  twist_body_msg_.twist.angular.x = twist_body_msg_.twist.angular.y =
      twist_body_msg_.twist.angular.z = 0;
  twist_global_msg_.twist.angular.x = twist_global_msg_.twist.angular.y =
      twist_global_msg_.twist.angular.z = 0;
  // Set ultrasonic properties
  ultrasonic_msg_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonic_msg_.field_of_view =
      0.785398;  // TODO defaulted to 45 degrees in radians; need real value
  ultrasonic_msg_.min_range = 0.1;
  ultrasonic_msg_.max_range = 8.0;
  // Set disparity image constants
  image_disparity_.delta_d = 0.0625;
  image_disparity_.valid_window.x_offset = 70;
  image_disparity_.valid_window.y_offset = 0;

  // Start configuring the Guidance
  e_sdk_err_code err_code = static_cast<e_sdk_err_code>(reset_config());
  RETURN_IF_ERR(err_code);

  err_code = static_cast<e_sdk_err_code>(init_transfer());
  RETURN_IF_ERR(err_code);

  // Check if all cameras are online
  int online[CAMERA_PAIR_NUM];
  err_code = static_cast<e_sdk_err_code>(get_online_status(online));
  RETURN_IF_ERR(err_code);

  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    if (online[i] != 1) {
      std::cout << "Error: Camera " << i << " offline.";
    }
  }

  err_code = static_cast<e_sdk_err_code>(get_stereo_cali(calibration_params));
  RETURN_IF_ERR(err_code);

  std::cout << "cu\tcv\tfocal\tbaseline\n";
  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    std::cout << calibration_params[i].cu << '\t' << calibration_params[i].cv
              << '\t' << calibration_params[i].focal << '\t'
              << calibration_params[i].baseline << std::endl;
  }

  // init multi-publishers and select data at the same time
  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    // init node handles
    depth_pnh_[i] = ros::NodeHandle(pnh_, "cam" + std::to_string(i) + "/depth");
    left_pnh_[i] = ros::NodeHandle(pnh_, "cam" + std::to_string(i) + "/left");
    right_pnh_[i] = ros::NodeHandle(pnh_, "cam" + std::to_string(i) + "/right");

    // init camera puplishers
    if (config.isDepthEnabled(i)) {
      createDepthPublisher(depth_pnh_[i], i,
                           "calibration_files/camera_params_left" + std::to_string(i) + ".ini");
      std::cout << "select depth" << std::endl;
      select_depth_image(static_cast<e_vbus_index>(i));
    }

    if (config.isCamEnabled(i, GuidanceConfiguration::cam_right)) {
      createImagePublisher(right_pnh_[i], i,
                           "calibration_files/camera_params_right" + std::to_string(i) + ".ini", false);
      std::cout << "select right" << std::endl;
      select_greyscale_image(static_cast<e_vbus_index>(i), CAM_RIGHT);
    }

    if (config.isCamEnabled(i, GuidanceConfiguration::cam_left)) {
      createImagePublisher(left_pnh_[i], i,
                           "calibration_files/camera_params_left"+std::to_string(i)+".ini", true);
      std::cout << "select left" << std::endl;
      select_greyscale_image(static_cast<e_vbus_index>(i), CAM_LEFT);
    }

    if (config.isDisparityEnabled(i)) {
      disparity_image_pub_[i] = pnh_.advertise<stereo_msgs::DisparityImage>(
          "cam" + std::to_string(i) + "/left/disparity", 1);
      std::cout << "select disp" << std::endl;
      select_disparity_image(static_cast<e_vbus_index>(i));
    }

    if (config.isUltrasonicEnabled()) {
      ultrasonic_pub_[i] =
          pnh_.advertise<sensor_msgs::Range>("sonar" + std::to_string(i), 10);
    }
  }

  if (config.isUltrasonicEnabled()) {
    ROS_INFO("select ultrasonic");
    select_ultrasonic();
  }

  if (config.isImuEnabled()) {
    imu_pub_ = pnh_.advertise<sensor_msgs::Imu>("imu", 10);
    ROS_INFO("select imu");
    select_imu();
  }

  if (config.isObstacleEnabled()) {
    obstacle_distance_pub_ =
        pnh_.advertise<sensor_msgs::LaserScan>("obstacle_distance", 10);
    ROS_INFO("select obstacle distance");
    select_obstacle_distance();
  }

  if (config.isVelocityEnabled()) {
    velocity_body_pub_ =
        pnh_.advertise<geometry_msgs::TwistStamped>("body/velocity", 10);
    ROS_INFO("select velocity");
    select_velocity();
  }

  if (config.isMotionEnabled()) {
    velocity_global_pub_ =
        pnh_.advertise<geometry_msgs::TwistStamped>("global/velocity", 10);
    pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("global/pose", 10);
    ROS_INFO("select motion");
    select_motion();
  }

  std::cout << "set event handler" << std::endl;
  err_code =
      static_cast<e_sdk_err_code>(set_sdk_event_handler(guidance_data_rcvd_cb));
  RETURN_IF_ERR(err_code);

  std::cout << "start transfer" << std::endl;
  err_code = static_cast<e_sdk_err_code>(start_transfer());
  RETURN_IF_ERR(err_code);

  return e_OK;
}

void GuidanceManager::stopTransfer() {
  std::lock_guard<std::mutex> guard(g_guidance_mutex);
  stop_transfer();
}

void GuidanceManager::releaseTransfer() {
  std::lock_guard<std::mutex> guard(g_guidance_mutex);
  release_transfer();
}

void GuidanceManager::createDepthPublisher(ros::NodeHandle nh,
                                           unsigned int index,
                                           std::string cam_info_path) {
  std::string idx = std::to_string(index);
  std::string cam_topic = "cam" + idx + "/depth/image_raw";
  std::string camera_name = "guidance/cam" + idx + "/depth";
  std::string cam_info_uri = "package://mrasl_guidance/" + cam_info_path;

  depth_cam_info_man[index] = new camera_info_manager::CameraInfoManager(
      nh, camera_name, cam_info_path);

  depth_cam_info_man[index]->loadCameraInfo(cam_info_uri);
  depth_image_pub_[index] =
      new image_transport::CameraPublisher(it_->advertiseCamera(cam_topic, 1));

  sensor_msgs::CameraInfo cinfo = depth_cam_info_man[index]->getCameraInfo();
  cinfo.P[2] = calibration_params[index].cu;
  cinfo.P[6] = calibration_params[index].cv;
  cinfo.P[0] = calibration_params[index].focal;
  cinfo.P[5] = calibration_params[index].focal;
  cinfo.P[3] = 0; // The depth image is on the left camera
  cinfo.P[7] = 0;
  depth_cam_info_man[index]->setCameraInfo(cinfo);
}

void GuidanceManager::createImagePublisher(ros::NodeHandle nh,
                                           unsigned int index,
                                           std::string cam_info_path,
                                           bool is_left) {
  std::string idx = std::to_string(index);
  std::string cam_topic = is_left ? "cam" + idx + "/left/image_raw"
                                  : "cam" + idx + "/right/image_raw";
  std::string camera_name = is_left
                                 ? "guidance/cam" + idx + "/left"
                                 : "guidance/cam" + idx + "/right";
  std::string cam_info_uri = "package://mrasl_guidance/" + cam_info_path;

  if (is_left) {
    left_cam_info_man[index] = new camera_info_manager::CameraInfoManager(
        nh, camera_name, cam_info_path);
    left_cam_info_man[index]->loadCameraInfo(cam_info_uri);
    left_image_pub_[index] = new image_transport::CameraPublisher(
        it_->advertiseCamera(cam_topic, 1));
    sensor_msgs::CameraInfo cinfo = left_cam_info_man[index]->getCameraInfo();
    cinfo.P[2] = calibration_params[index].cu;
    cinfo.P[6] = calibration_params[index].cv;
    cinfo.P[0] = calibration_params[index].focal;
    cinfo.P[5] = calibration_params[index].focal;
    cinfo.P[3] = 0; // The depth image is on the left camera
    cinfo.P[7] = 0;
    left_cam_info_man[index]->setCameraInfo(cinfo);
  } else {
    right_cam_info_man[index] = new camera_info_manager::CameraInfoManager(
        nh, camera_name, cam_info_path);
    right_cam_info_man[index]->loadCameraInfo(cam_info_uri);
    right_image_pub_[index] = new image_transport::CameraPublisher(
        it_->advertiseCamera(cam_topic, 1));
    sensor_msgs::CameraInfo cinfo = right_cam_info_man[index]->getCameraInfo();
    cinfo.P[2] = calibration_params[index].cu;
    cinfo.P[6] = calibration_params[index].cv;
    cinfo.P[0] = calibration_params[index].focal;
    cinfo.P[5] = calibration_params[index].focal;
    cinfo.P[3] = -calibration_params[index].focal * calibration_params[index].baseline; // Tx
    cinfo.P[7] = 0; // Ty
    right_cam_info_man[index]->setCameraInfo(cinfo);
  }
}

e_sdk_err_code GuidanceManager::configureGuidance(void) {
  stop_transfer();
  reset_config();

  e_sdk_err_code err_code = static_cast<e_sdk_err_code>(start_transfer());
  RETURN_IF_ERR(err_code);
}

cv::Mat depth8(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
void GuidanceManager::image_handler(int data_len, char *content) {
  // Figure out what kind of image this is and which camera it came from
  image_data *data = (image_data *)content;

  // time stamping
  if (timestamp_buf_.count(data->frame_index) < 1) {
    // didn't get this frame index yet
    TimeStamp t;
    t.frame_index = data->frame_index;
    t.time_stamp = data->time_stamp;
    t.rostime = ros::Time::now();
    timestamp_buf_[data->frame_index] = t;
  }
  ros::Time time = timestamp_buf_[data->frame_index].rostime;

  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    if (data->m_greyscale_image_left[i] != NULL) {
      memcpy(image_left_.image.data, data->m_greyscale_image_left[i], IMG_SIZE);
      image_left_.header.frame_id = "cam" + std::to_string(i) + "_left";
      image_left_.header.stamp = time;
      image_left_.header.seq = data->frame_index;
      image_left_.encoding = sensor_msgs::image_encodings::MONO8;

      sensor_msgs::CameraInfoPtr ci_left(
          new sensor_msgs::CameraInfo(left_cam_info_man[i]->getCameraInfo()));
      ci_left->header.stamp = image_left_.header.stamp;
      ci_left->header.frame_id = "cam" + std::to_string(i) + "_left";

      left_image_pub_[i]->publish(image_left_.toImageMsg(), ci_left);
      // std::cout << "left " << data->frame_index << '\t' << data->time_stamp
      // << std::endl;
    }
    if (data->m_greyscale_image_right[i] != NULL) {
      memcpy(image_right_.image.data, data->m_greyscale_image_right[i],
             IMG_SIZE);
      image_right_.header.frame_id = "cam" + std::to_string(i) + "_right";
      image_right_.header.stamp = time;
      image_right_.header.seq = data->frame_index;
      image_right_.encoding = sensor_msgs::image_encodings::MONO8;

      sensor_msgs::CameraInfoPtr ci_right(
          new sensor_msgs::CameraInfo(right_cam_info_man[i]->getCameraInfo()));
      ci_right->header.stamp = image_right_.header.stamp;
      ci_right->header.frame_id = image_right_.header.frame_id;
      ci_right->header.seq = data->frame_index;
      right_image_pub_[i]->publish(image_right_.toImageMsg(), ci_right);
    }
    if (data->m_depth_image[i] != NULL) {
      // 16 bit signed images, omitting processing here
      memcpy(mat_depth16_.data, data->m_depth_image[i], IMG_SIZE * 2);

      cv::filterSpeckles(mat_depth16_, -16, maxSpeckleSize_, maxSpeckleDiff_);
      mat_depth16_.convertTo(mat_depth16_, CV_32FC1);
      cv::medianBlur(mat_depth16_, mat_depth16_, 3);
      image_depth_.image = mat_depth16_ / 128.0;
      image_depth_.header.frame_id = "cam" + std::to_string(i) + "_left";
      image_depth_.header.stamp = time;
      image_depth_.header.seq = data->frame_index;
      image_depth_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

      sensor_msgs::CameraInfoPtr ci_depth(
          new sensor_msgs::CameraInfo(depth_cam_info_man[i]->getCameraInfo()));

      ci_depth->header.stamp = image_depth_.header.stamp;
      ci_depth->header.frame_id = image_depth_.header.frame_id;
      ci_depth->header.seq = data->frame_index;
      depth_image_pub_[i]->publish(image_depth_.toImageMsg(), ci_depth);
      mat_depth16_.convertTo(mat_depth16_, CV_16SC1);
    }
    if (data->m_disparity_image[i] != NULL) {
      memcpy(image_cv_disparity16_.image.data, data->m_disparity_image[i],
             IMG_SIZE * 2);
      image_cv_disparity16_.image.convertTo(image_cv_disparity32_.image, CV_32FC1);
      image_disparity_.image = *image_cv_disparity16_.toImageMsg();
      image_disparity_.header.frame_id = "cam" + std::to_string(i) + "_left";
      image_disparity_.header.stamp = time;
      image_disparity_.header.seq = data->frame_index;
      image_disparity_.f = calibration_params[i].focal;
      image_disparity_.T = calibration_params[i].baseline;
      disparity_image_pub_[i].publish(image_disparity_);
    }
  }
}

void GuidanceManager::imu_handler(int data_len, char *content) {
  imu *imu_data = (imu *)content;
  imu_msg_.header.frame_id = "imu";
  // time stamping
  if (timestamp_buf_.count(imu_data->frame_index) < 1) {
    // didn't get this frame index yet
    TimeStamp t;
    t.frame_index = imu_data->frame_index;
    t.time_stamp = imu_data->time_stamp;
    t.rostime = ros::Time::now();
    timestamp_buf_[imu_data->frame_index] = t;
  }
  ros::Time time = timestamp_buf_[imu_data->frame_index].rostime;
  imu_msg_.header.seq = imu_data->frame_index;
  imu_msg_.header.stamp = time;
  imu_msg_.linear_acceleration.x = imu_data->acc_x * GRAVITY;
  imu_msg_.linear_acceleration.y = imu_data->acc_y * GRAVITY;
  imu_msg_.linear_acceleration.z = imu_data->acc_z * GRAVITY;
  imu_msg_.orientation.w = imu_data->q[0];
  imu_msg_.orientation.x = imu_data->q[1];
  imu_msg_.orientation.y = imu_data->q[2];
  imu_msg_.orientation.z = imu_data->q[3];
  imu_pub_.publish(imu_msg_);
}

void GuidanceManager::ultrasonic_handler(int data_len, char *content) {
  ultrasonic_data *ultrasonic = (ultrasonic_data *)content;
  ultrasonic_msg_.header.stamp = ros::Time::now();
  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    ultrasonic_msg_.header.frame_id = "sonar" + std::to_string(i);
    ultrasonic_msg_.range = 0.001f * ultrasonic->ultrasonic[i];
    ultrasonic_pub_[i].publish(ultrasonic_msg_);
  }
}

void GuidanceManager::motion_handler(int data_len, char *content) {
  motion *m = (motion *)content;
  ros::Time now = ros::Time::now();
  pose_msg_.header.frame_id = "local_origin";
  pose_msg_.header.stamp = now;
  pose_msg_.pose.position.x = m->position_in_global_x;
  pose_msg_.pose.position.y = m->position_in_global_y;
  pose_msg_.pose.position.z = m->position_in_global_z;
  pose_msg_.pose.orientation.w = m->q0;
  pose_msg_.pose.orientation.x = m->q1;
  pose_msg_.pose.orientation.y = m->q2;
  pose_msg_.pose.orientation.z = m->q3;
  pose_pub_.publish(pose_msg_);

  twist_global_msg_.header.frame_id = "local_origin";
  twist_global_msg_.header.stamp = now;
  twist_global_msg_.twist.linear.x = m->velocity_in_global_x;
  twist_global_msg_.twist.linear.y = m->velocity_in_global_y;
  twist_global_msg_.twist.linear.z = m->velocity_in_global_z;
  velocity_global_pub_.publish(twist_global_msg_);
  /*
     static tf2_ros::TransformBroadcaster br;
     geometry_msgs::TransformStamped transformStamped;

     transformStamped.header.stamp = ros::Time::now();
     transformStamped.header.frame_id = "world";
     transformStamped.child_frame_id = "guidance";
     transformStamped.transform.translation.x = m->position_in_global_x;
     transformStamped.transform.translation.y = m->position_in_global_y;
     transformStamped.transform.translation.z = m->position_in_global_z;
     transformStamped.transform.rotation.x = m->q1;
     transformStamped.transform.rotation.y = m->q2;
     transformStamped.transform.rotation.z = m->q3;
     transformStamped.transform.rotation.w = m->q0;

     br.sendTransform(transformStamped);*/
}

void GuidanceManager::velocity_handler(int data_len, char *content) {
  velocity *vo = (velocity *)content;
  twist_body_msg_.header.frame_id = "guidance";
  twist_body_msg_.header.stamp = ros::Time::now();
  twist_body_msg_.twist.linear.x = 0.001f * vo->vx;
  twist_body_msg_.twist.linear.y = 0.001f * vo->vy;
  twist_body_msg_.twist.linear.z = 0.001f * vo->vz;
  velocity_body_pub_.publish(twist_body_msg_);
}

void GuidanceManager::obstacle_handler(int data_len, char *content) {}

void GuidanceManager::cleanTimestampBuf() {
  if (timestamp_buf_.size() > 25) timestamp_buf_.erase(timestamp_buf_.begin());
}

int guidance_data_rcvd_cb(int event, int data_len, char *content) {
  std::lock_guard<std::mutex> guard(g_guidance_mutex);
  if (content == NULL) return 1;

  switch (event) {
    case e_image:
      GuidanceManager::getInstance()->image_handler(data_len, content);
      break;
    case e_imu:
      GuidanceManager::getInstance()->imu_handler(data_len, content);
      break;
    case e_velocity:
      GuidanceManager::getInstance()->velocity_handler(data_len, content);
      break;
    case e_obstacle_distance:
      GuidanceManager::getInstance()->obstacle_handler(data_len, content);
      break;
    case e_ultrasonic:
      GuidanceManager::getInstance()->ultrasonic_handler(data_len, content);
      break;
    case e_motion:
      GuidanceManager::getInstance()->motion_handler(data_len, content);
      break;
    default:
      break;
  }

  GuidanceManager::getInstance()->cleanTimestampBuf();
}
