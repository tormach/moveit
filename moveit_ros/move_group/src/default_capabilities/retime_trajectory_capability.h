//
// Created by alexander on 12/16/20.
//

/* Author: Alexander Rössler */

#pragma once

#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/RetimeTrajectory.h>

namespace move_group
{
class MoveGroupRetimeTrajectoryService : public MoveGroupCapability
{
public:
  MoveGroupRetimeTrajectoryService();

  void initialize() override;

private:
  bool computeService(moveit_msgs::RetimeTrajectory::Request& req, moveit_msgs::RetimeTrajectory::Response& res);

  ros::ServiceServer retime_trajectory_service_;
};
}  // namespace move_group
