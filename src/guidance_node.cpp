#include <csignal>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <dji/utils.h>
#include <dji/guidance.h>
#include <mrasl/MissionPlannerRequest.h>
#include <mrasl/UpdateNodeStatus.h>
#include <Definitions.h>
#include <mrasl_guidance/guidanceConfig.h>
#include "guidance_manager.hpp"

GuidanceManager *GuidanceManager::s_instance_ = 0;
ros::ServiceClient mission_planner_client;

ros::Timer * shutdown_timer;


void shutdown_timerCallback(const ros::TimerEvent&){
  ROS_WARN("Received Kill Signal.");

  mrasl::UpdateNodeStatus srv;
  srv.request.node_ID = mrasl::node_ID::GUIDANCE;
  srv.request.node_status = mrasl::node_status::STATUS_STOP;
  if(!mission_planner_client.call(srv))
  {
    ROS_ERROR("Unable to contact mission planner.");
  }

  ros::shutdown();
}

void signal_handler(int signal) {
  GuidanceManager::getInstance()->stopTransfer();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  GuidanceManager::getInstance()->releaseTransfer();
  mrasl::UpdateNodeStatus srv;
  srv.request.node_ID = mrasl::node_ID::GUIDANCE;
  srv.request.node_status = mrasl::node_status::STATUS_STOP;
  if(!mission_planner_client.call(srv))
  {
    ROS_ERROR("Unable to contact mission planner.");
  }
  ros::shutdown();
}

void config_callback(guidance::guidanceConfig &config, uint32_t level) {
  GuidanceManager::getInstance()->set_maxDiff(config.maxDiff);
  GuidanceManager::getInstance()->set_maxSpeckleSize(config.maxSpeckleSize);
  GuidanceManager::getInstance()->set_maxDiffCpu(config.maxDiffCpu);
  GuidanceManager::getInstance()->set_maxSpeckleSize(config.maxSpeckleSizeCpu);
}

bool missionPlannerCallback(mrasl::MissionPlannerRequest::Request &req,
                            mrasl::MissionPlannerRequest::Response &res)
{
  ROS_INFO("Guidance mp req %d, id %d, stat %d", (int) req.node_request,
           (int) req.node_ID, 0);
    if(req.node_request == mrasl::REQUEST_ABORT ||
       req.node_request == mrasl::REQUEST_STOP) {
        GuidanceManager::getInstance()->stopTransfer();

        GuidanceManager::getInstance()->releaseTransfer();
        shutdown_timer->setPeriod(ros::Duration(2.0),true);
        shutdown_timer->start();
    }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "guidance_node", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, signal_handler);
  ros::NodeHandle nh("~");

  shutdown_timer = new ros::Timer(nh.createTimer(ros::Duration(2.0),shutdown_timerCallback,true,false));

  mission_planner_client = nh.serviceClient<mrasl::UpdateNodeStatus>("/mission_main/update_node_status");
  dynamic_reconfigure::Server<guidance::guidanceConfig> server;
  dynamic_reconfigure::Server<guidance::guidanceConfig>::CallbackType f;

  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);

  ros::ServiceServer mission_planner_service =
          nh.advertiseService("/mission_main/guidance_request", missionPlannerCallback);
  if (GuidanceManager::getInstance()->init(nh)) {
    std::cout << "Uh oh..." << std::endl;
    return 1;
  }

  ros::spin();

  return 0;
}
