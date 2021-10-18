#ifndef _DWA_PLANNER_H_
#define _DWA_PLANNER_H_

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <ros/publisher.h>
#include <nav_msgs/Path.h>
#include "dwa_planner/utils.h"

namespace dwa_planner
{
class DWAPlanner
{
public:
  DWAPlanner(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
             double inscribed_radius, double circumscribed_radius, const DWAConfig& cfg, ros::NodeHandle nh);
  virtual ~DWAPlanner();

  bool computeVelocityCommands(const Velocity& robot_vel, const Pose2D& robot_pose,
                               const std::vector<Pose2D>& global_plan, unsigned char const* const* costmap, int size_x,
                               int size_y, double resolution, double origin_x, double origin_y, Velocity& cmd_vel);

private:
  /**
   * @brief Check whether the planned path is feasible or not.
   *
   * This method currently checks only that the path, or a part of the path is collision free.
   * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
   * @return \c true, if the robot footprint along the first part of the path intersects with
   *         any obstacle in the costmap, \c false otherwise.
   */
  bool isPathFeasible(const std::vector<Pose2D>& path);
  void publishCandidatePaths(const std::vector<std::vector<Pose2D> >& candidate_paths);

private:
  const DWAConfig* cfg_;  //!< Config class that stores and manages all related parameters

  base_local_planner::CostmapModel* costmap_model_;   //!< Pointer to the costmap model
  std::vector<geometry_msgs::Point> footprint_spec_;  //!< The specification of the footprint of the robot in world
                                                      //!< coordinates
  double inscribed_radius_;                           //!< The radius of the inscribed circle of the robot
  double circumscribed_radius_;                       //!< The radius of the circumscribed circle of the robot

  ros::Publisher candidate_paths_pub_;
};

}  // namespace dwa_planner

#endif  // _DWA_PLANNER_H_