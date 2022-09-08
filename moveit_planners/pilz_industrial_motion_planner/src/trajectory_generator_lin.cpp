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

#include "pilz_industrial_motion_planner/trajectory_generator_lin.h"

#include <cassert>
#include <ros/ros.h>
#include <sstream>
#include <time.h>

#include <moveit/robot_state/conversions.h>

#include <kdl/path_line.hpp>
#include <utility>
#include <kdl/utilities/error.h>

#include <tf2_kdl/tf2_kdl.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pilz_industrial_motion_planner/trajectory_segment.hpp"

namespace pilz_industrial_motion_planner
{
TrajectoryGeneratorLIN::TrajectoryGeneratorLIN(const moveit::core::RobotModelConstPtr& robot_model,
                                               const LimitsContainer& planner_limits, const std::string& /*group_name*/,
                                               std::shared_ptr<ErrorDetailsContainer> error_details,
                                               std::shared_ptr<PlanningParameters> planning_parameters)
  : TrajectoryGenerator::TrajectoryGenerator(robot_model, planner_limits, std::move(error_details), std::move(planning_parameters))
{
  if (!planner_limits_.hasFullCartesianLimits())
  {
    ROS_ERROR("Cartesian limits not set for LIN trajectory generator.");
    throw TrajectoryGeneratorInvalidLimitsException("Cartesian limits are not fully set for LIN trajectory generator.");
  }
}

void TrajectoryGeneratorLIN::extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                                                   const planning_interface::MotionPlanRequest& req,
                                                   TrajectoryGenerator::MotionPlanInfo& info) const
{
  ROS_DEBUG("Extract necessary information from motion plan request.");

  info.group_name = req.group_name;
  std::string frame_id{ robot_model_->getModelFrame() };

  // goal given in joint space
  if (!req.goal_constraints.front().joint_constraints.empty())
  {
    info.link_name = robot_model_->getJointModelGroup(req.group_name)->getSolverInstance()->getTipFrame();

    if (req.goal_constraints.front().joint_constraints.size() !=
        robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames().size())
    {
      std::ostringstream os;
      os << "Number of joints in goal does not match number of joints of group "
            "(Number joints goal: "
         << req.goal_constraints.front().joint_constraints.size() << " | Number of joints of group: "
         << robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames().size() << ")";
      throw JointNumberMismatch(os.str());
    }
    // initializing all joints of the model
    for (const auto& joint_name : robot_model_->getVariableNames())
    {
      info.goal_joint_position[joint_name] = 0;
    }

    for (const auto& joint_item : req.goal_constraints.front().joint_constraints)
    {
      info.goal_joint_position[joint_item.joint_name] = joint_item.position;
    }

    // Ignored return value because at this point the function should always
    // return 'true'.
    computeLinkFK(robot_model_, info.link_name, info.goal_joint_position, info.goal_pose);
  }
  // goal given in Cartesian space
  else
  {
    info.link_name = req.goal_constraints.front().position_constraints.front().link_name;
    if (req.goal_constraints.front().position_constraints.front().header.frame_id.empty() ||
        req.goal_constraints.front().orientation_constraints.front().header.frame_id.empty())
    {
      ROS_WARN("Frame id is not set in position/orientation constraints of "
               "goal. Use model frame as default");
      frame_id = robot_model_->getModelFrame();
    }
    else
    {
      frame_id = req.goal_constraints.front().position_constraints.front().header.frame_id;
    }
    info.goal_pose = getConstraintPose(req.goal_constraints.front());
  }

  assert(req.start_state.joint_state.name.size() == req.start_state.joint_state.position.size());
  for (const auto& joint_name : robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames())
  {
    auto it{ std::find(req.start_state.joint_state.name.cbegin(), req.start_state.joint_state.name.cend(), joint_name) };
    if (it == req.start_state.joint_state.name.cend())
    {
      std::ostringstream os;
      os << "Could not find joint \"" << joint_name << "\" of group \"" << req.group_name
         << "\" in start state of request";
      throw LinJointMissingInStartState(os.str());
    }
    size_t index = it - req.start_state.joint_state.name.cbegin();
    info.start_joint_position[joint_name] = req.start_state.joint_state.position[index];
  }

  // Ignored return value because at this point the function should always
  // return 'true'.
  computeLinkFK(robot_model_, info.link_name, info.start_joint_position, info.start_pose);

  if (!planning_parameters_->getTrimOnFailure())
  {
    // check goal pose ik before Cartesian motion plan starts
    std::map<std::string, double> ik_solution;
    if (!computePoseIK(scene, info.group_name, info.link_name, info.goal_pose, frame_id, info.start_joint_position,
                       ik_solution))
    {
      std::ostringstream os;
      os << "Failed to compute inverse kinematics for link: " << info.link_name << " of goal pose";
      throw LinInverseForGoalIncalculable(os.str());
    }
  }
}

void TrajectoryGeneratorLIN::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                  const planning_interface::MotionPlanRequest& req, const MotionPlanInfo& plan_info,
                                  const double & /*sampling_time*/, trajectory_msgs::JointTrajectory& joint_trajectory)
{
  // create Cartesian path for lin
  std::unique_ptr<KDL::Path> path(setPathLIN(plan_info.start_pose, plan_info.goal_pose));
  planning_interface::MotionPlanRequest new_req = req;
  moveit_msgs::MoveItErrorCodes error_code;
  bool scaling_factor_corrected = false;
  bool succeeded = false;

  const double sampling_time = planning_parameters_->getSamplingTime();
  const double sampling_distance = planning_parameters_->getSamplingDistance();
  const double min_scaling_correction_factor = planning_parameters_->getMinScalingCorrectionFactor();
  const bool strict_limits = planning_parameters_->getStrictLimits();
  const bool trim_on_failure = planning_parameters_->getTrimOnFailure();
  const bool output_tcp_joints = planning_parameters_->getOutputTcpJoints();

  if (trim_on_failure && !output_tcp_joints) {
    ROS_WARN("trim_on_failure only supported when output_tcp_joints is active");
  }

  while (!succeeded)
  {
    std::pair<double, double> max_scaling_factors { 1.0, 1.0 };
    // create velocity profile
    std::unique_ptr<pilz_industrial_motion_planner::VelocityProfile> vp(
        cartesianTrapVelocityProfile(new_req.max_velocity_scaling_factor, new_req.max_acceleration_scaling_factor, path, new_req.duration));

    // calculate sampling_time at constant velocity
    const double const_sampling_time = sampling_distance / vp->maxVelocity();

    // combine path and velocity profile into Cartesian trajectory
    // with the third parameter set to false, KDL::Trajectory_Segment does not
    // take
    // the ownership of Path and Velocity Profile
    pilz_industrial_motion_planner::Trajectory_Segment cart_trajectory(path.get(), vp.get(), false);

    //  sample the Cartesian trajectory and compute joint trajectory using inverse
    //  kinematics
    Eigen::Isometry3d pose_sample_last;
    if (!generateJointTrajectory(scene, planner_limits_.getJointLimitContainer(), cart_trajectory, plan_info.group_name,
                                 plan_info.link_name, plan_info.start_joint_position, sampling_time, const_sampling_time,
                                 joint_trajectory, error_code, max_scaling_factors, pose_sample_last, false, output_tcp_joints))
    {
      if (trim_on_failure && !joint_trajectory.points.empty())
      {
        // trimming active and at least one trajectory point, need to resample
        path = setPathLIN(plan_info.start_pose, pose_sample_last);
        ROS_DEBUG_STREAM("Shortened trajectory");
      }
      else if (error_code.val != moveit_msgs::MoveItErrorCodes::PLANNING_FAILED)
      {
        break; // error not related to limit violation
      }
      else if (strict_limits) {
        break; // planning failed due to joint velocity/acceleration violation
      }

      const double new_scaling_factor = std::min(new_req.max_velocity_scaling_factor * max_scaling_factors.first,
                                                 new_req.max_acceleration_scaling_factor * max_scaling_factors.second);
      new_req.max_velocity_scaling_factor = new_scaling_factor;
      new_req.max_acceleration_scaling_factor = new_scaling_factor;
      if (new_scaling_factor < min_scaling_correction_factor)
      {
        ROS_INFO_STREAM("Joint velocity or acceleration limit violated and below minimum scaling factor.");
        break; // would require scaling factor below threshold
      }

      ROS_DEBUG_STREAM("updating scaling factors " <<
                       new_req.max_velocity_scaling_factor << " " << new_req.max_acceleration_scaling_factor);
      scaling_factor_corrected = true;
      continue;
    }

    succeeded = true;
  }

  if (!succeeded) {
    joint_trajectory.points.clear();
    throw LinTrajectoryConversionFailure("Failed to generate valid joint trajectory from the Cartesian path",
                                         error_code.val);
  }

  if (scaling_factor_corrected) {
    ROS_INFO_STREAM("Joint velocity or acceleration limit violated.\nScaling factors have been corrected to vel: " <<
                     new_req.max_velocity_scaling_factor << " accel: " << new_req.max_acceleration_scaling_factor);
  }
}

std::unique_ptr<KDL::Path> TrajectoryGeneratorLIN::setPathLIN(const Eigen::Affine3d& start_pose,
                                                              const Eigen::Affine3d& goal_pose) const
{
  ROS_DEBUG("Set Cartesian path for LIN command.");

  KDL::Frame kdl_start_pose, kdl_goal_pose;
  tf2::fromMsg(tf2::toMsg(start_pose), kdl_start_pose);
  tf2::fromMsg(tf2::toMsg(goal_pose), kdl_goal_pose);
  double eqradius = planner_limits_.getCartesianLimits().getMaxTranslationalVelocity() /
                    planner_limits_.getCartesianLimits().getMaxRotationalVelocity();
  KDL::RotationalInterpolation* rot_interpo = new KDL::RotationalInterpolation_SingleAxis();

  return std::unique_ptr<KDL::Path>(new KDL::Path_Line(kdl_start_pose, kdl_goal_pose, rot_interpo, eqradius, true));
}

}  // namespace pilz_industrial_motion_planner
