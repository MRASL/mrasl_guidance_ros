#include "guidance_configuration.hpp"
#include <mrasl_guidance/guidanceConfig.h>

void GuidanceConfiguration::applyFromNodeHandle(ros::NodeHandle nh) {
  // Since the node starts a dynamic reconfigure server, I suppose the
  // parameters will be set
  nh.getParam("enable_imu", enable_imu_);
  nh.getParam("enable_utrasonic", enable_utrasonic_);
  nh.getParam("enable_obstacle_distance", enable_obstacle_distance_);
  nh.getParam("enable_velocity", enable_velocity_);
  nh.getParam("enable_motion", enable_motion_);

  nh.getParam("camera_front/enable_depth", depth_enabled_[front]);
  nh.getParam("camera_left/enable_depth", depth_enabled_[left]);
  nh.getParam("camera_back/enable_depth", depth_enabled_[back]);
  nh.getParam("camera_right/enable_depth", depth_enabled_[right]);
  nh.getParam("camera_bottom/enable_depth", depth_enabled_[bottom]);

  nh.getParam("camera_front/enable_disparity", disparity_enabled_[front]);
  nh.getParam("camera_left/enable_disparity", disparity_enabled_[right]);
  nh.getParam("camera_back/enable_disparity", disparity_enabled_[back]);
  nh.getParam("camera_right/enable_disparity", disparity_enabled_[left]);
  nh.getParam("camera_bottom/enable_disparity", disparity_enabled_[bottom]);

  nh.getParam("camera_front/enable_left", cam_enabled_[front][cam_left]);
  nh.getParam("camera_left/enable_left", cam_enabled_[right][cam_left]);
  nh.getParam("camera_back/enable_left", cam_enabled_[back][cam_left]);
  nh.getParam("camera_right/enable_left", cam_enabled_[left][cam_left]);
  nh.getParam("camera_bottom/enable_left", cam_enabled_[bottom][cam_left]);

  nh.getParam("camera_front/enable_right", cam_enabled_[front][cam_right]);
  nh.getParam("camera_left/enable_right", cam_enabled_[right][cam_right]);
  nh.getParam("camera_back/enable_right", cam_enabled_[back][cam_right]);
  nh.getParam("camera_right/enable_right", cam_enabled_[left][cam_right]);
  nh.getParam("camera_bottom/enable_right", cam_enabled_[bottom][cam_right]);
}

void GuidanceConfiguration::applyConfig(guidance::guidanceConfig config) {
  enable_imu_ = config.enable_imu;
  enable_utrasonic_ = config.enable_utrasonic;
  enable_obstacle_distance_ = config.enable_obstacle_distance;
  enable_velocity_ = config.enable_velocity;
  enable_motion_ = config.enable_motion;

  depth_enabled_[front] = config.enable_depth_front;
  depth_enabled_[right] = config.enable_depth_right;
  depth_enabled_[back] = config.enable_depth_back;
  depth_enabled_[left] = config.enable_depth_left;
  depth_enabled_[bottom] = config.enable_depth_bottom;
  disparity_enabled_[front] = config.enable_disparity_front;
  disparity_enabled_[right] = config.enable_disparity_right;
  disparity_enabled_[back] = config.enable_disparity_back;
  disparity_enabled_[left] = config.enable_disparity_left;
  disparity_enabled_[bottom] = config.enable_disparity_bottom;
  cam_enabled_[front][cam_left] = config.enable_camera_front_left;
  cam_enabled_[right][cam_left] = config.enable_camera_right_left;
  cam_enabled_[back][cam_left] = config.enable_camera_back_left;
  cam_enabled_[left][cam_left] = config.enable_camera_left_left;
  cam_enabled_[bottom][cam_left] = config.enable_camera_bottom_left;
  cam_enabled_[front][cam_right] = config.enable_camera_front_right;
  cam_enabled_[right][cam_right] = config.enable_camera_right_right;
  cam_enabled_[back][cam_right] = config.enable_camera_back_right;
  cam_enabled_[left][cam_right] = config.enable_camera_left_right;
  cam_enabled_[bottom][cam_right] = config.enable_camera_bottom_right;
}
