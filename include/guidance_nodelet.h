#include "ros/ros.h"
#include <nodelet/nodelet.h>

namespace mrasl_guidance
{
  class GuidanceNodelet : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  };
}
