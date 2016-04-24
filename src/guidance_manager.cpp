#include <dji/guidance.h>
#include "guidance_manager.hpp"
#include <opencv2/opencv.hpp>

GuidanceManager::GuidanceManager(const ros::NodeHandle &pnh) :
    pnh_(pnh) {
  image_left_.create(IMG_WIDTH, IMG_HEIGHT, CV_8UC1);
  image_right_.create(IMG_WIDTH, IMG_HEIGHT, CV_8UC1);
  image_depth_.create(IMG_WIDTH, IMG_HEIGHT, CV_16SC1);
}

GuidanceManager::~GuidanceManager() {
  // Not sure this is actually useful
  image_left_.release();
  image_right_.release();
  image_depth_.release();
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}

dji::e_sdk_err_code GuidanceManager::init() {
  dji::reset_config();
  RETURN_IF_ERR(err_code);

  // Check if all cameras are online
  int online[CAMERA_PAIR_NUM];
  err_code = get_online_status(online);

  for(int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    if (online[i] != 1) {
      std::cout << "Error: Camera " << i << " offline.";
    }
  }

  // get all calibration parameters
  for(int i = 0; o < CAMERA_PAIR_NUM; ++i) {

  }
}
