#include "dji/utils.h"

namespace dji {

DJI_lock::DJI_lock() { pthread_mutex_init(&m_lock, NULL); }

DJI_lock::~DJI_lock() {}

void DJI_lock::enter() { pthread_mutex_lock(&m_lock); }

void DJI_lock::leave() { pthread_mutex_unlock(&m_lock); }

DJI_event::DJI_event() { sem_init(&m_sem, 0, 0); }

DJI_event::~DJI_event() {}

int DJI_event::set_event() {
  int ret = sem_post(&m_sem);
  return ret;
}

int DJI_event::wait_event() {
  int ret = sem_wait(&m_sem);
  return ret;
}

/**
 * @brief Convert a DJI guidance error code to a verbose string
 */
const char *err_code_str(e_sdk_err_code e) {
  switch (e) {
    case e_timeout:
      return "Operation timed out";
    case e_libusb_io_err:
      return "libusb IO error";
    case e_OK:
      return "Success";
    case e_load_libusb_err:
      return "Load libusb library error";
    case e_sdk_not_inited:
      return "SDK software is not ready";
    case e_hardware_not_ready:
      return "Guidance hardware is not ready";
    case e_disparity_not_allowed:
      return "Disparity or depth image is not allowed to be selected";
    case e_image_frequency_not_allowed:
      return "Image frequency must be one of the enum type "
             "e_image_data_frequency";
    case e_config_not_ready:
      return "Config is not ready";
    case e_online_flag_not_ready:
      return "Online flag is not ready";
    case e_stereo_cali_not_ready:
      return "Stereo calibration parameters are not ready";
    default:
      return "Unknown error";
  }
}

}  // namespace dji
