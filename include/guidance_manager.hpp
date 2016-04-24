#ifndef GUIDANCE_MANAGER_HPP
#define GUIDANCE_MANAGER_HPP

#include <ros/ros.h>
#include <dji/guidance.h>
#include <opencv2/opencv.hpp>



class GuidanceManager {
public:
  GuidanceManager(const ros::NodeHandle& pnh);
  ~GuidanceManager();

  dji::e_sdk_err_code enable_imu();
  dji::e_sdk_err_code enable_ultrasonic();
  dji::e_sdk_err_code enable_obstacle_distance();
  dji::e_sdk_err_code enable_velocity();
  dji::e_sdk_err_code enable_motion();
  dji::e_sdk_err_code enable_greyscale_image(dji::e_vbus_index cam_id);
  dji::e_sdk_err_code enable_dept_image(dji::e_vbus_index cam_id);

  dji::e_sdk_err_code disable_imu();
  dji::e_sdk_err_code disable_ultrasonic();
  dji::e_sdk_err_code disable_obstacle_distance();
  dji::e_sdk_err_code disable_velocity();
  dji::e_sdk_err_code disable_motion();
  dji::e_sdk_err_code disable_greyscale_image(dji::e_vbus_index cam_id);
  dji::e_sdk_err_code disable_dept_image(dji::e_vbus_index cam_id);
private:
  ros::NodeHandle pnh_;

#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define IMG_SIZE (IMG_WIDTH * IMG_HEIGHT)

  // Buffer images
  cv::Mat image_left_;
  cv::Mat image_right_;
  cv::Mat image_depth_;

  dji::stereo_cali calibration_params[5];

  dji::e_sdk_err_code init();

};

#endif // GUIDANCE_MANAGER_HPP
