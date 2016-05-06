#ifndef GUIDANCE_MANAGER_HPP
#define GUIDANCE_MANAGER_HPP
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <dji/guidance.h>
#include "guidance_configuration.hpp"
#include <opencv2/opencv.hpp>

/**
 * @brief The GuidanceManager class
 * @note Due to the C nature of the DJI library, some stuff in this class had to
 * be static.
 * Because of this, might as well follow a singleton pattern and hopefully
 * everything works out.
 *
 * RIP my computer engineering degree
 */
class GuidanceManager {
 public:
  // Singleton pattern
  static GuidanceManager *getInstance() {
    if (s_instance_ == NULL) s_instance_ = new GuidanceManager;
    return s_instance_;
  }

  // static void setNodeHandle(ros::NodeHandle pnh) { pnh_ = pnh; }
  e_sdk_err_code init(ros::NodeHandle pnh);

  e_sdk_err_code enable_imu();
  e_sdk_err_code enable_ultrasonic();
  e_sdk_err_code enable_obstacle_distance();
  e_sdk_err_code enable_velocity();
  e_sdk_err_code enable_motion();
  e_sdk_err_code enable_greyscale_image(e_vbus_index cam_id);
  e_sdk_err_code enable_depth_image(e_vbus_index cam_id);

  e_sdk_err_code disable_imu();
  e_sdk_err_code disable_ultrasonic();
  e_sdk_err_code disable_obstacle_distance();
  e_sdk_err_code disable_velocity();
  e_sdk_err_code disable_motion();
  e_sdk_err_code disable_greyscale_image(e_vbus_index cam_id);
  e_sdk_err_code disable_depth_image(e_vbus_index cam_id);

  // Alright at this point I have to admit this is spaghetti
  // code with extra meatballs
  void image_handler(int data_len, char *content);
  void imu_handler(int data_len, char *content);
  void ultrasonic_handler(int data_len, char *content);
  void motion_handler(int data_len, char *content);
  void velocity_handler(int data_len, char *content);
  void obstacle_handler(int data_len, char *content);

  // parameters
  void set_maxSpeckleSize(int maxSpeckleSize) {
    maxSpeckleSize_ = maxSpeckleSize;
  };
  void set_maxDiff(double maxDiff) { maxSpeckleDiff_ = maxDiff; };

 private:
  ros::NodeHandle pnh_;
  static GuidanceManager *s_instance_;
  GuidanceManager() {}  // Purposely hide the damn things
  GuidanceManager(GuidanceManager const &);
  void operator=(GuidanceManager const &);

#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define IMG_SIZE 76800
#define CAMERA_PAIR_NUM 5

  // Camera stuff
  stereo_cali calibration_params[5];
  camera_info_manager::CameraInfoManager *right_cam_info_man[CAMERA_PAIR_NUM];
  camera_info_manager::CameraInfoManager *left_cam_info_man[CAMERA_PAIR_NUM];
  camera_info_manager::CameraInfoManager *depth_cam_info_man[CAMERA_PAIR_NUM];

  /**
   * Creates a depth image Publisher
   * @param nh            The parent nodehandle of the depth image Publisher
   * @param index         Camera index of the Guidance
   * @param cam_info_path Path to the camera info file
   */
  void createDepthPublisher(ros::NodeHandle nh, unsigned int index,
                            std::string cam_info_path);

  /**
   * Creates an image Publisher
   * @param nh            The parent nodehandle of the image Publisher
   * @param index         Camera index of the guidance
   * @param cam_info_path Path to the camera info file
   * @param is_left       Boolean, if true, this is the left cam, else, right
   * cam
   */
  void createImagePublisher(ros::NodeHandle nh, unsigned int index,
                            std::string cam_info_path, bool is_left);

  /**
   * Apply a configuration to the Guidance
   */
  e_sdk_err_code configureGuidance(void);

  // image processing stuff
  int maxSpeckleSize_;
  double maxSpeckleDiff_;

  // Message buffers
  cv_bridge::CvImage image_left_;
  cv_bridge::CvImage image_right_;
  cv_bridge::CvImage image_depth_;
  cv::Mat mat_depth16_;
  cv_bridge::CvImage image_cv_disparity_;
  stereo_msgs::DisparityImage image_disparity_;
  sensor_msgs::Range ultrasonic_msg_;
  sensor_msgs::Imu imu_msg_;
  geometry_msgs::TwistStamped twist_body_msg_;
  geometry_msgs::TwistStamped twist_global_msg_;
  geometry_msgs::PoseStamped pose_msg_;

  /**
   * @brief guidance_data_rcvd_cb
   * Call back to dispatch data to the right handler
   * Some C/C++ hackery going on here
   */
  // static int guidance_data_rcvd_cb(int event, int data_len, char *content);

  // publishers
  image_transport::ImageTransport *it_;
  image_transport::CameraPublisher *depth_image_pub_[CAMERA_PAIR_NUM];
  image_transport::CameraPublisher *left_image_pub_[CAMERA_PAIR_NUM];
  image_transport::CameraPublisher *right_image_pub_[CAMERA_PAIR_NUM];
  ros::Publisher disparity_image_pub_[CAMERA_PAIR_NUM];
  ros::Publisher imu_pub_;
  ros::Publisher obstacle_distance_pub_;
  ros::Publisher velocity_body_pub_;
  ros::Publisher velocity_global_pub_;
  ros::Publisher ultrasonic_pub_[CAMERA_PAIR_NUM];
  ros::Publisher pose_pub_;
};

#endif  // GUIDANCE_MANAGER_HPP
