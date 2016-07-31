#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>

#include "guidance_nodelet.h"

PLUGINLIB_EXPORT_CLASS(mrasl_guidance::GuidanceNodelet, nodelet::Nodelet);

GuidanceManager *GuidanceManager::s_instance_ = 0;
namespace mrasl_guidance
{
  void GuidanceNodelet::onInit() {
    NODELET_DEBUG("Initializing GuidanceNodelet...");
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    if (GuidanceManager::getInstance()->init(private_nh)) {
      NODELET_FATAL("Couldn't init GuidanceManager.");
    }
  }
}
