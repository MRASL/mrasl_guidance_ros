#ifndef GUIDANCE_NODELET_H
#define GUIDANCE_NODELET_H
#include <csignal>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <mrasl_guidance/guidanceConfig.h>
#include "guidance_manager.hpp"

namespace mrasl_guidance
{
  class GuidanceNodelet : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:

  };
}

PLUGINLIB_EXPORT_CLASS(mrasl_guidance::GuidanceNodelet, nodelet::Nodelet);

#endif // GUIDANCE_NODELET_H
