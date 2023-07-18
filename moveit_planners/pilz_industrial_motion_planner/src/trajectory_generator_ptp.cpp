/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "pilz_industrial_motion_planner/trajectory_generator_ptp.h"
#include "moveit/robot_state/conversions.h"
#include "ros/ros.h"

#include <iostream>
#include <sstream>
#include <utility>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pilz_industrial_motion_planner
{
TrajectoryGeneratorPTP::TrajectoryGeneratorPTP(const robot_model::RobotModelConstPtr& robot_model,
                                               const LimitsContainer& planner_limits, const std::string& group_name,
                                               std::shared_ptr<ErrorDetailsContainer> error_details,
                                               std::shared_ptr<PlanningParameters> planning_parameters)
  : TrajectoryGenerator::TrajectoryGenerator(robot_model, planner_limits, std::move(error_details), std::move(planning_parameters))
{
  if (!planner_limits_.hasJointLimits())
  {
    throw TrajectoryGeneratorInvalidLimitsException("joint limit not set");
  }

  joint_limits_ = planner_limits_.getJointLimitContainer();

  // collect most strict joint limits for each group in robot model
  const auto* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
    throw TrajectoryGeneratorInvalidLimitsException("invalid group: " + group_name);

  const auto& active_joints = jmg->getActiveJointModelNames();

  // no active joints
  if (!active_joints.empty())
  {
    most_strict_limit_ = joint_limits_.getCommonLimit(active_joints);

    if (!most_strict_limit_.has_velocity_limits)
      throw TrajectoryGeneratorInvalidLimitsException("velocity limit not set for group " + group_name);
    if (!most_strict_limit_.has_acceleration_limits)
      throw TrajectoryGeneratorInvalidLimitsException("acceleration limit not set for group " + group_name);
    if (!most_strict_limit_.has_deceleration_limits)
      throw TrajectoryGeneratorInvalidLimitsException("deceleration limit not set for group " + group_name);
  }

  ROS_INFO("Initialized Point-to-Point Trajectory Generator.");
}

void TrajectoryGeneratorPTP::planPTP(const std::map<std::string, double>& start_pos,
                                     const std::map<std::string, double>& goal_pos,
                                     trajectory_msgs::JointTrajectory& joint_trajectory,
                                     const double& velocity_scaling_factor, const double& acceleration_scaling_factor,
                                     const double& sampling_time, const double& duration)
{
  // initialize joint names
  for (const auto& item : goal_pos)
  {
    joint_trajectory.joint_names.push_back(item.first);
  }

  // check if goal already reached
  bool goal_reached = true;
  for (auto const& goal : goal_pos)
  {
    if (fabs(start_pos.at(goal.first) - goal.second) >= MIN_MOVEMENT)
    {
      goal_reached = false;
      break;
    }
  }
  if (goal_reached)
  {
    ROS_INFO_STREAM("Goal already reached, set one goal point explicitly.");
    if (joint_trajectory.points.empty())
    {
      trajectory_msgs::JointTrajectoryPoint point;
      point.time_from_start = ros::Duration(sampling_time);
      for (const std::string& joint_name : joint_trajectory.joint_names)
      {
        point.positions.push_back(start_pos.at(joint_name));
        point.velocities.push_back(0);
        point.accelerations.push_back(0);
      }
      joint_trajectory.points.push_back(point);
    }
    return;
  }

  // compute the fastest trajectory and choose the slowest accel, const and decel segments for the leading axis
  double max_duration;
  std::map<std::string, VelocityProfileATrap> velocity_profile;
  double max_acc_time = 0.0;
  double max_const_time = 0.0;
  double max_dec_time = 0.0;
  for (const auto& joint_name : joint_trajectory.joint_names)
  {
    // create velocity profile if necessary
    velocity_profile.insert(std::make_pair(
        joint_name, VelocityProfileATrap(velocity_scaling_factor * joint_limits_.getLimit(joint_name).max_velocity,
                                         acceleration_scaling_factor * joint_limits_.getLimit(joint_name).max_acceleration,
                                         acceleration_scaling_factor * joint_limits_.getLimit(joint_name).max_deceleration)));

    velocity_profile.at(joint_name).SetProfileDuration(start_pos.at(joint_name), goal_pos.at(joint_name), duration);
    if (velocity_profile.at(joint_name).firstPhaseDuration() > max_acc_time) {
      max_acc_time = velocity_profile.at(joint_name).firstPhaseDuration();
    }
    if (velocity_profile.at(joint_name).secondPhaseDuration() > max_const_time) {
      max_const_time = velocity_profile.at(joint_name).secondPhaseDuration();
    }
    if (velocity_profile.at(joint_name).thirdPhaseDuration() > max_dec_time) {
      max_dec_time = velocity_profile.at(joint_name).thirdPhaseDuration();
    }
  }
  max_duration = max_acc_time + max_const_time + max_dec_time;

  // Full Synchronization
  // This should only work if all axes have same max_vel, max_acc, max_dec
  // values
  // reset the velocity profile for other joints
  //double acc_time = velocity_profile.at(leading_axis).firstPhaseDuration();
  //double const_time = velocity_profile.at(leading_axis).secondPhaseDuration();
  //double dec_time = velocity_profile.at(leading_axis).thirdPhaseDuration();

  for (const auto& joint_name : joint_trajectory.joint_names)
  {
    //if (joint_name != leading_axis)
    //{
      // make full synchronization
      // causes the program to terminate if acc_time<=0 or dec_time<=0 (should
      // be prevented by goal_reached block above)
      // by using the most strict limit, the following should always return true
      if (!velocity_profile.at(joint_name)
               .setProfileAllDurations(start_pos.at(joint_name), goal_pos.at(joint_name), max_acc_time, max_const_time,
                                       max_dec_time))
      // LCOV_EXCL_START
      {
        std::stringstream error_str;
        error_str << "TrajectoryGeneratorPTP::planPTP(): Can not synchronize "
                     "velocity profile of axis "
                  << joint_name;
        throw PtpVelocityProfileSyncFailed(error_str.str());
      }
      // LCOV_EXCL_STOP
    //}
  }

  // first generate the time samples
  std::vector<double> time_samples;
  for (double t_sample = 0.0; t_sample < max_duration; t_sample += sampling_time)
  {
    time_samples.push_back(t_sample);
  }
  // add last time
  time_samples.push_back(max_duration);

  // construct joint trajectory point
  for (double time_stamp : time_samples)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(time_stamp);
    for (std::string& joint_name : joint_trajectory.joint_names)
    {
      point.positions.push_back(velocity_profile.at(joint_name).Pos(time_stamp));
      point.velocities.push_back(velocity_profile.at(joint_name).Vel(time_stamp));
      point.accelerations.push_back(velocity_profile.at(joint_name).Acc(time_stamp));
    }
    joint_trajectory.points.push_back(point);
  }

  // Set last point velocity and acceleration to zero
  std::fill(joint_trajectory.points.back().velocities.begin(), joint_trajectory.points.back().velocities.end(), 0.0);
  std::fill(joint_trajectory.points.back().accelerations.begin(), joint_trajectory.points.back().accelerations.end(),
            0.0);
}

void TrajectoryGeneratorPTP::extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                                                   const planning_interface::MotionPlanRequest& req,
                                                   MotionPlanInfo& info) const
{
  info.group_name = req.group_name;

  // extract start state information
  info.start_joint_position.clear();
  for (std::size_t i = 0; i < req.start_state.joint_state.name.size(); ++i)
  {
    info.start_joint_position[req.start_state.joint_state.name[i]] = req.start_state.joint_state.position[i];
  }

  // extract goal
  info.goal_joint_position.clear();
  if (!req.goal_constraints.at(0).joint_constraints.empty())
  {
    for (const auto& joint_constraint : req.goal_constraints.at(0).joint_constraints)
    {
      info.goal_joint_position[joint_constraint.joint_name] = joint_constraint.position;
    }
  }
  // solve the IK for the goal, we use a separate planning group with global solving enabled
  else
  {
    Eigen::Isometry3d goal_pose = getConstraintPose(req.goal_constraints.front());
    if (!computePoseIK(scene, req.group_name + "_global", req.goal_constraints.at(0).position_constraints.at(0).link_name,
                       goal_pose, robot_model_->getModelFrame(), info.start_joint_position, info.goal_joint_position))
    {
      throw PtpNoIkSolutionForGoalPose("No IK solution for goal pose");
    }
  }
}

void TrajectoryGeneratorPTP::plan(const planning_scene::PlanningSceneConstPtr& /*scene*/,
                                  const planning_interface::MotionPlanRequest& req, const MotionPlanInfo& plan_info,
                                  const double& /*sampling_time*/, trajectory_msgs::JointTrajectory& joint_trajectory)
{
  // plan the ptp trajectory
  const double sampling_time = planning_parameters_->getSamplingTime();
  planPTP(plan_info.start_joint_position, plan_info.goal_joint_position, joint_trajectory,
          req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor, sampling_time, req.duration);
}

}  // namespace pilz_industrial_motion_planner
