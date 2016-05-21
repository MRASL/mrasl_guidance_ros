#ifndef GUIDANCE_CONFIGURATION_HPP
#define GUIDANCE_CONFIGURATION_HPP
#include <ros/ros.h>
#include <mrasl_guidance/guidanceConfig.h>

class GuidanceConfiguration {
 public:
  GuidanceConfiguration();
  static constexpr unsigned int cam_right = 0;
  static constexpr unsigned int cam_left = 1;
  static constexpr unsigned int num_cams = 2;
  static constexpr unsigned int bottom = 0;
  static constexpr unsigned int front = 1;
  static constexpr unsigned int left = 2;
  static constexpr unsigned int back = 3;
  static constexpr unsigned int right = 4;
  static constexpr unsigned int num_cam_pairs = 5;

  void applyFromNodeHandle(ros::NodeHandle nh);

  void applyConfig(guidance::guidanceConfig config);

  /**
   * Check if a certain cam is enabled
   * @param  pair Which module (bottom, front, left, back, right)
   * @param  cam  Which camera left or right
   * @return      If that it is enabled
   */
  bool isCamEnabled(unsigned int pair, unsigned int cam) {
    return cam_enabled_[pair][cam];
  }

  bool isDepthEnabled(unsigned int pair) { return depth_enabled_[pair]; }
  bool isDisparityEnabled(unsigned int pair) {
    return disparity_enabled_[pair];
  }

  bool isImuEnabled() { return enable_imu_; }
  bool isUltrasonicEnabled() { return enable_utrasonic_; }
  bool isObstacleEnabled() { return enable_obstacle_distance_; }
  bool isVelocityEnabled() { return enable_velocity_; }
  bool isMotionEnabled() { return enable_motion_; }

 private:
  bool cam_enabled_[num_cam_pairs][num_cams];
  bool depth_enabled_[num_cam_pairs];
  bool disparity_enabled_[num_cam_pairs];
  bool enable_imu_;
  bool enable_utrasonic_;
  bool enable_obstacle_distance_;
  bool enable_velocity_;
  bool enable_motion_;
};

#endif  // GUIDANCE_CONFIGURATION_HPP
