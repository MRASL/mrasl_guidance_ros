#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <guidance/guidanceConfig.h>

void config_callback(guidance::guidanceConfig &config, uint32_t level) {}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "guidance_node");

  dynamic_reconfigure::Server<guidance::guidanceConfig> server;
  dynamic_reconfigure::Server<guidance::guidanceConfig>::CallbackType f;

  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);

  return 0;
}
