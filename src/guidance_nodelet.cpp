#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>

#include "guidance_nodelet.h"

PLUGINLIB_EXPORT_CLASS(mrasl_guidance::GuidanceNodelet, nodelet::Nodelet);

namespace mrasl_guidance
{
  void GuidanceNodelet::onInit() {
    NODELET_DEBUG("Initializing GuidanceNodelet...");
  }
}
