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
