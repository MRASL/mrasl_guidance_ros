#ifndef GUIDANCE_MANAGER_HPP
#define GUIDANCE_MANAGER_HPP
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <dji/guidance.h>
#include <opencv2/opencv.hpp>

/**
 * @brief The GuidanceManager class
 * @note Due to the C nature of the DJI library, some stuff in this class had to
 * be static.
 * Because of this, might as well follow a singleton pattern and hopefully
 * everything works out.
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

  // Message buffers
  cv_bridge::CvImage image_left_;
  cv_bridge::CvImage image_right_;
  cv_bridge::CvImage image_depth_;
  cv_bridge::CvImage image_cv_disparity_;
  stereo_msgs::DisparityImage image_disparity_;
  stereo_cali calibration_params[5];
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
  image_transport::Publisher depth_image_pub_[CAMERA_PAIR_NUM];
  image_transport::Publisher left_image_pub_[CAMERA_PAIR_NUM];
  image_transport::Publisher right_image_pub_[CAMERA_PAIR_NUM];
  ros::Publisher disparity_image_pub_[CAMERA_PAIR_NUM];
  ros::Publisher imu_pub_;
  ros::Publisher obstacle_distance_pub_;
  ros::Publisher velocity_body_pub_;
  ros::Publisher velocity_global_pub_;
  ros::Publisher ultrasonic_pub_;
  ros::Publisher pose_pub_;
};

#endif  // GUIDANCE_MANAGER_HPP
