#ifndef DUMMY_GLOBAL_PLANNER_H_
#define DUMMY_GLOBAL_PLANNER_H_

#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

namespace dummy_global_planner {

class DummyGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
  DummyGlobalPlanner();
  DummyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  void pathCallback(const nav_msgs::Path::ConstPtr& msg);

private:
  ros::Subscriber path_sub_;
  std::vector<geometry_msgs::PoseStamped> last_plan_;
  bool initialized_;
};

} // namespace dummy_global_planner

#endif
