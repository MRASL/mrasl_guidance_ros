#include <string>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <dji/guidance.h>

#include "guidance_manager.hpp"
#include <opencv2/opencv.hpp>

#define CAM_LEFT true
#define CAM_RIGHT false

ros::NodeHandle GuidanceManager::pnh_;
int guidance_data_rcvd_cb(int event, int data_len,
                                           char *content);

GuidanceManager::GuidanceManager()
    : it_(image_transport::ImageTransport(GuidanceManager::pnh_)) {
  image_left_.create(IMG_WIDTH, IMG_HEIGHT, CV_8UC1);
  image_right_.create(IMG_WIDTH, IMG_HEIGHT, CV_8UC1);
  image_depth_.create(IMG_WIDTH, IMG_HEIGHT, CV_16SC1);

  // init image publishers
  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    depth_image_pub_[i] =
        it_.advertise("~/cam" + std::to_string(i) + "/depth", 1);
    left_image_pub_[i] =
        it_.advertise("~/cam" + std::to_string(i) + "/left", 1);
    right_image_pub_[i] =
        it_.advertise("~/cam" + std::to_string(i) + "/right", 1);
    disparity_image_pub_[i] =
        it_.advertise("~/cam" + std::to_string(i) + "/disparity", 1);
  }

  imu_pub_ = pnh_.advertise<sensor_msgs::Imu>("~/imu", 10);
  obstacle_distance_pub_ =
      pnh_.advertise<sensor_msgs::LaserScan>("~/obstacle_distance", 10);
  velocity_pub_ = pnh_.advertise<sensor_msgs::Imu>("~/velocity", 10);
  ultrasonic_pub_ = pnh_.advertise<sensor_msgs::LaserScan>("~/sonar", 10);
  position_pub_ =
      pnh_.advertise<geometry_msgs::Vector3Stamped>("~/position", 10);
}

GuidanceManager::~GuidanceManager() {
  // Not sure this is actually useful
  image_left_.release();
  image_right_.release();
  image_depth_.release();
}

#define RETURN_IF_ERR(err_code)                                                \
  {                                                                            \
    if (err_code) {                                                            \
      release_transfer();                                                      \
      std::cout << "Error: " << (e_sdk_err_code)err_code << " at " << __LINE__ \
                << "," << __FILE__ << std::endl;                               \
      return err_code;                                                         \
    }                                                                          \
  }

e_sdk_err_code GuidanceManager::init() {
  e_sdk_err_code err_code = static_cast<e_sdk_err_code>(reset_config());
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

  select_imu();
  select_ultrasonic();
  select_obstacle_distance();
  select_velocity();
  select_motion();

  select_greyscale_image(e_vbus1, CAM_LEFT);
  select_greyscale_image(e_vbus1, CAM_RIGHT);
  select_depth_image(e_vbus1);
  select_disparity_image(e_vbus1);

  err_code = static_cast<e_sdk_err_code>(
      set_sdk_event_handler(guidance_data_rcvd_cb));
  RETURN_IF_ERR(err_code);

  err_code = static_cast<e_sdk_err_code>(start_transfer());
  RETURN_IF_ERR(err_code);
}

void GuidanceManager::image_handler(int data_len, char *content) {
  // Figure out what kind of image this is and which camera it came from
  image_data* data = (image_data*)content;
  //if (data->m_depth_image[data->frame_index])

}

void GuidanceManager::imu_handler(int data_len, char *content) {

}

void GuidanceManager::ultrasonic_handler(int data_len, char *content) {

}

void GuidanceManager::motion_handler(int data_len, char *content) {

}

void GuidanceManager::velocity_handler(int data_len, char *content) {

}

void GuidanceManager::obstacle_handler(int data_len, char *content) {

}

int guidance_data_rcvd_cb(int event, int data_len,
                                           char *content) {
  // TODO add mutex?
  if (content == NULL) return 1;

  switch (event) {
    case e_image:
      GuidanceManager::getInstance().image_handler(data_len, content);
      break;
    case e_imu:
      GuidanceManager::getInstance().imu_handler(data_len, content);
      break;
    case e_velocity:
      GuidanceManager::getInstance().velocity_handler(data_len, content);
      break;
    case e_obstacle_distance:
      GuidanceManager::getInstance().obstacle_handler(data_len, content);
      break;
    case e_ultrasonic:
      GuidanceManager::getInstance().ultrasonic_handler(data_len, content);
      break;
    case e_motion:
      GuidanceManager::getInstance().motion_handler(data_len, content);
      break;
    default:
      break;
  }
}
