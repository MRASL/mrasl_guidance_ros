#ifndef GUIDANCE_MANAGER_HPP
#define GUIDANCE_MANAGER_HPP
#include <map>

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
#include <opencv2/gpu/gpu.hpp>

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
  void stopTransfer();
  void releaseTransfer();

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
  void image_handler(int data_len, char *content, ros::Time timestamp);
  void imu_handler(int data_len, char *content, ros::Time timestamp);
  void ultrasonic_handler(int data_len, char *content, ros::Time timestamp);
  void motion_handler(int data_len, char *content, ros::Time timestamp);
  void velocity_handler(int data_len, char *content, ros::Time timestamp);
  void obstacle_handler(int data_len, char *content, ros::Time timestamp);

  /**
   *  Pop off old time stamps that we don't need
   */
  void cleanTimestampBuf();
  ros::Time getTimestamp(header const *head);

  // parameters
  void set_maxSpeckleSize(int maxSpeckleSize) {
    maxSpeckleSize_ = maxSpeckleSize;
  };
  void set_maxDiff(double maxDiff) { maxSpeckleDiff_ = maxDiff; };

 private:
#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define IMG_SIZE 76800
#define CAMERA_PAIR_NUM 5

  ros::NodeHandle pnh_;
  ros::NodeHandle depth_pnh_[CAMERA_PAIR_NUM];
  ros::NodeHandle disp_pnh_[CAMERA_PAIR_NUM];
  ros::NodeHandle left_pnh_[CAMERA_PAIR_NUM];
  ros::NodeHandle right_pnh_[CAMERA_PAIR_NUM];
  static GuidanceManager *s_instance_;
  GuidanceManager();  // Purposely hide the damn things
  GuidanceManager(GuidanceManager const &);
  void operator=(GuidanceManager const &);

  // Camera stuff
  stereo_cali calibration_params[CAMERA_PAIR_NUM];
  cv::Mat Q[CAMERA_PAIR_NUM];
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
   * Sends an image pair to the GPU to get block matched. The resulting depth image
   * is kept as a private attribute;
   * @param index         Camera index of the guidance
   */
  void gpuBM(unsigned int index);

  /**
   * Apply a configuration to the Guidance
   */
  e_sdk_err_code configureGuidance(void);

  GuidanceConfiguration config;

  // image processing stuff
  int maxSpeckleSize_;
  double maxSpeckleDiff_;
  cv::StereoBM* sbm_cpu;
  /*cv::gpu::StereoConstantSpaceBP* sbm;
  cv::gpu::GpuMat gpu_left_, gpu_right_, gpu_depth_, gpu_buf_, gpu_buf16_;*/
  unsigned int sbm_idx_[CAMERA_PAIR_NUM]; // this contains the last frame_index rcvd for a certain cam pair
  double disp2depth_const_[CAMERA_PAIR_NUM];

  // Message buffers
  cv_bridge::CvImage image_left_;
  cv_bridge::CvImage image_right_;
  cv_bridge::CvImage image_depth_;
  cv_bridge::CvImage image_gpubm_buf_left_[CAMERA_PAIR_NUM];
  cv_bridge::CvImage image_gpubm_buf_right_[CAMERA_PAIR_NUM];
  cv::Mat mat_depth16_;
  cv_bridge::CvImage image_cv_disparity16_, image_cv_disparity32_;
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

  typedef struct TimeStamp_ {
    unsigned int frame_index;
    unsigned int time_stamp;
    ros::Time rostime;
  } TimeStamp;

  std::map<unsigned int, TimeStamp> timestamp_buf_;
};

#endif  // GUIDANCE_MANAGER_HPP
