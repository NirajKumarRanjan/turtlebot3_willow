#include "dummy_global_planner/dummy_global_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dummy_global_planner::DummyGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace dummy_global_planner {

DummyGlobalPlanner::DummyGlobalPlanner() : initialized_(false) {}

DummyGlobalPlanner::DummyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void DummyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle nh("~/" + name);
    path_sub_ = nh.subscribe("/planned_path", 1, &DummyGlobalPlanner::pathCallback, this);
    initialized_ = true;
    ROS_INFO("Dummy Global Planner initialized and listening to /planned_path");
  }
}

void DummyGlobalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
  last_plan_.clear();
  for (auto pose : msg->poses) {
    last_plan_.push_back(pose);
  }
}

bool DummyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan) {
  plan = last_plan_;
  ROS_INFO("Dummy Global Planner returning path with %lu points", plan.size());
  return !plan.empty();
}

} // namespace dummy_global_planner
