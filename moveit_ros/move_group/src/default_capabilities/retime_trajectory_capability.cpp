/* Author: Alexander Rössler */

#include "retime_trajectory_capability.h"
#include <moveit/move_group/capability_names.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace move_group
{
  MoveGroupRetimeTrajectoryService::MoveGroupRetimeTrajectoryService()
      : MoveGroupCapability("RetimeTrajectoryService")
  {
  }

  void MoveGroupRetimeTrajectoryService::initialize()
  {
    retime_trajectory_service_ = root_node_handle_.advertiseService(RETIME_TRAJECTORY_SERVICE_NAME,
                                                                 &MoveGroupRetimeTrajectoryService::computeService, this);
  }

  bool MoveGroupRetimeTrajectoryService::computeService(moveit_msgs::RetimeTrajectory::Request& req,
                                                     moveit_msgs::RetimeTrajectory::Response& res)
  {
    ROS_INFO_NAMED(getName(), "Received request to retime trajectory");
    context_->planning_scene_monitor_->updateFrameTransforms();

    moveit::core::RobotState robot_state =
        planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();

    if (const moveit::core::JointModelGroup* jmg = robot_state.getJointModelGroup(req.group_name))
    {
      robot_trajectory::RobotTrajectory traj_obj(context_->planning_scene_monitor_->getRobotModel(), req.group_name);
      traj_obj.setRobotTrajectoryMsg(robot_state, req.trajectory);

      bool algorithm_found = true;
      // Do the actual retiming
      if (req.algorithm == "iterative_time_parameterization")
      {
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        time_param.computeTimeStamps(traj_obj, req.velocity_scaling_factor, req.acceleration_scaling_factor);
      }
      else if (req.algorithm == "iterative_spline_parameterization")
      {
        trajectory_processing::IterativeSplineParameterization time_param;
        time_param.computeTimeStamps(traj_obj, req.velocity_scaling_factor, req.acceleration_scaling_factor);
      }
      else if (req.algorithm == "time_optimal_trajectory_generation")
      {
        trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
        time_param.computeTimeStamps(traj_obj, req.velocity_scaling_factor, req.acceleration_scaling_factor);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(getName(), "Unknown time parameterization algorithm: " << req.algorithm);
        algorithm_found = false;
        res.trajectory = moveit_msgs::RobotTrajectory();
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      }

      if (algorithm_found) {
        // Convert the retimed trajectory back into a message
        traj_obj.getRobotTrajectoryMsg(res.trajectory);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      }
    }
    else
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;

    return true;
  }
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupRetimeTrajectoryService, move_group::MoveGroupCapability)
