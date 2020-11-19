#include "move_pguard/local/a_local_planner.h"

#include <ros/ros.h>

namespace dr = dynamic_reconfigure;

namespace move_pguard
{
namespace local
{
ALocalPlanner::ALocalPlanner() : cfg_server_(nullptr)
{
}

ALocalPlanner::~ALocalPlanner()
{
}

void ALocalPlanner::initializeConfigServer(std::string name)
{
  ros::NodeHandle n("~" + name);
  // Instantiate the server and set its callback
  cfg_server_ = std::shared_ptr<dr::Server<LocalPlannerConfig>>(new dr::Server<LocalPlannerConfig>(n));
  auto callback = boost::bind(&ALocalPlanner::configCallback, this, _1, _2);
  cfg_server_->setCallback(callback);
}

}  // namespace local
}  // namespace move_pguard
