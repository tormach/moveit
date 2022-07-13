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

#include "pilz_industrial_motion_planner/trajectory_functions.h"

#include <moveit/planning_scene/planning_scene.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <kdl/rotational_interpolation.hpp>
#include <kdl/rotational_interpolation_sa.hpp>

#include "pilz_industrial_motion_planner/velocity_profile.h"

bool pilz_industrial_motion_planner::computePoseIK(const planning_scene::PlanningSceneConstPtr& scene,
                                                   const std::string& group_name, const std::string& link_name,
                                                   const Eigen::Isometry3d& pose, const std::string& frame_id,
                                                   const std::map<std::string, double>& seed,
                                                   std::map<std::string, double>& solution, bool check_self_collision,
                                                   const double timeout)
{
  const moveit::core::RobotModelConstPtr& robot_model = scene->getRobotModel();
  if (!robot_model->hasJointModelGroup(group_name))
  {
    ROS_ERROR_STREAM("Robot model has no planning group named as " << group_name);
    return false;
  }

  if (!robot_model->getJointModelGroup(group_name)->canSetStateFromIK(link_name))
  {
    ROS_ERROR_STREAM("No valid IK solver exists for " << link_name << " in planning group " << group_name);
    return false;
  }

  if (frame_id != robot_model->getModelFrame())
  {
    ROS_ERROR_STREAM("Given frame (" << frame_id << ") is unequal to model frame(" << robot_model->getModelFrame()
                                     << ")");
    return false;
  }

  robot_state::RobotState rstate(robot_model);
  // By setting the robot state to default values, we basically allow
  // the user of this function to supply an incomplete or even empty seed.
  rstate.setToDefaultValues();
  rstate.setVariablePositions(seed);

  moveit::core::GroupStateValidityCallbackFn ik_constraint_function;
  ik_constraint_function = [check_self_collision, scene](moveit::core::RobotState* robot_state,
                                                         const moveit::core::JointModelGroup* joint_group,
                                                         const double* joint_group_variable_values) {
    return pilz_industrial_motion_planner::isStateColliding(check_self_collision, scene, robot_state, joint_group,
                                                            joint_group_variable_values);
  };

  // call ik
  if (rstate.setFromIK(robot_model->getJointModelGroup(group_name), pose, link_name, timeout, ik_constraint_function))
  {
    // copy the solution
    for (const auto& joint_name : robot_model->getJointModelGroup(group_name)->getActiveJointModelNames())
    {
      solution[joint_name] = rstate.getVariablePosition(joint_name);
    }
    return true;
  }
  else
  {
    ROS_INFO_STREAM("Inverse kinematics for pose \n" << pose.translation() << " " << pose.rotation() << "\n has no solution.");
    return false;
  }
}

bool pilz_industrial_motion_planner::computePoseIK(const planning_scene::PlanningSceneConstPtr& scene,
                                                   const std::string& group_name, const std::string& link_name,
                                                   const geometry_msgs::Pose& pose, const std::string& frame_id,
                                                   const std::map<std::string, double>& seed,
                                                   std::map<std::string, double>& solution, bool check_self_collision,
                                                   const double timeout)
{
  Eigen::Isometry3d pose_eigen;
  tf2::fromMsg(pose, pose_eigen);
  return computePoseIK(scene, group_name, link_name, pose_eigen, frame_id, seed, solution, check_self_collision,
                       timeout);
}

bool pilz_industrial_motion_planner::computeLinkFK(const moveit::core::RobotModelConstPtr& robot_model,
                                                   const std::string& link_name,
                                                   const std::map<std::string, double>& joint_state,
                                                   Eigen::Isometry3d& pose)
{  // create robot state
  robot_state::RobotState rstate(robot_model);

  // check the reference frame of the target pose
  if (!rstate.knowsFrameTransform(link_name))
  {
    ROS_ERROR_STREAM("The target link " << link_name << " is not known by robot.");
    return false;
  }

  // set the joint positions
  rstate.setToDefaultValues();
  rstate.setVariablePositions(joint_state);

  // update the frame
  rstate.update();
  pose = rstate.getFrameTransform(link_name);

  return true;
}

bool pilz_industrial_motion_planner::verifySampleJointLimits(
    const std::map<std::string, double>& position_last, const std::map<std::string, double>& velocity_last,
    const std::map<std::string, double>& position_current, double duration_last, double duration_current,
    const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits,
    std::pair<double, double>& max_scaling_factors)
{
  const double epsilon = 10e-6;
  if (duration_current <= epsilon)
  {
    ROS_ERROR("Sample duration too small, cannot compute the velocity");
    max_scaling_factors.first = 0.0;
    max_scaling_factors.second = 0.0;
    return false;
  }

  double velocity_current, acceleration_current;
  double velocity_scale = 1.0;
  double acceleration_scale = 1.0;
  bool succeeded = true;

  for (const auto& pos : position_current)
  {
    velocity_current = (pos.second - position_last.at(pos.first)) / duration_current;

    if (!joint_limits.verifyVelocityLimit(pos.first, velocity_current))
    {
      velocity_scale = std::min(fabs(joint_limits.getLimit(pos.first).max_velocity) / fabs(velocity_current), velocity_scale);
      ROS_DEBUG_STREAM("Joint velocity limit of " << pos.first << " violated. Set the velocity scaling factor lower!"
                                                  << " Actual joint velocity is " << velocity_current
                                                  << ", while the limit is "
                                                  << joint_limits.getLimit(pos.first).max_velocity << ". ");
      succeeded = false;
    }

    acceleration_current = (velocity_current - velocity_last.at(pos.first)) / (duration_last + duration_current) * 2.0;
    // acceleration case
    if (fabs(velocity_last.at(pos.first)) <= fabs(velocity_current))
    {
      if (joint_limits.getLimit(pos.first).has_acceleration_limits &&
          fabs(acceleration_current) > fabs(joint_limits.getLimit(pos.first).max_acceleration))
      {
        acceleration_scale = std::min(fabs(joint_limits.getLimit(pos.first).max_acceleration)/ fabs(acceleration_current), acceleration_scale);
        ROS_DEBUG_STREAM("Joint acceleration limit of "
                         << pos.first << " violated. Set the acceleration scaling factor lower!"
                         << " Actual joint acceleration is " << acceleration_current << ", while the limit is "
                         << joint_limits.getLimit(pos.first).max_acceleration << ". ");
        succeeded = false;
      }
    }
    // deceleration case
    else
    {
      if (joint_limits.getLimit(pos.first).has_deceleration_limits &&
          fabs(acceleration_current) > fabs(joint_limits.getLimit(pos.first).max_deceleration))
      {
        acceleration_scale = std::min(fabs(joint_limits.getLimit(pos.first).max_deceleration)/ fabs(acceleration_current), acceleration_scale);
        ROS_DEBUG_STREAM("Joint deceleration limit of "
                         << pos.first << " violated. Set the acceleration scaling factor lower!"
                         << " Actual joint deceleration is " << acceleration_current << ", while the limit is "
                         << joint_limits.getLimit(pos.first).max_deceleration << ". ");
        succeeded = false;
      }
    }
  }

  max_scaling_factors.first = velocity_scale;
  max_scaling_factors.second = acceleration_scale;
  return succeeded;
}

bool pilz_industrial_motion_planner::generateJointTrajectory(
    const planning_scene::PlanningSceneConstPtr& scene,
    const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits, const pilz_industrial_motion_planner::Trajectory_Segment& trajectory,
    const std::string& group_name, const std::string& link_name,
    const std::map<std::string, double>& initial_joint_position, const double& sampling_time,
    trajectory_msgs::JointTrajectory& joint_trajectory, moveit_msgs::MoveItErrorCodes& error_code,
    std::pair<double, double>& max_scaling_factors, Eigen::Isometry3d &pose_sample_last, bool check_self_collision,
    bool output_tcp_joints, bool strict_limits, double min_scaling_correction_factor)
{
  ROS_DEBUG("Generate joint trajectory from a Cartesian trajectory.");
  const auto old_max_scaling_factors = max_scaling_factors;

  const moveit::core::RobotModelConstPtr& robot_model = scene->getRobotModel();
  ros::Time generation_begin = ros::Time::now();

  // generate the time samples
  const double epsilon = 10e-06;  // avoid adding the last time sample twice
  std::vector<double> time_samples;
  const double t_first_transition = trajectory.getVelocityProfile()->firstPhaseDuration();
  const double t_second_transition = t_first_transition + trajectory.getVelocityProfile()->secondPhaseDuration();
  bool first_transition_added = false;
  bool second_transition_added = false;
  for (double t_sample = 0.0; t_sample < trajectory.Duration() - epsilon; t_sample += sampling_time)
  {
    if (!first_transition_added && (t_sample > t_first_transition)) {
      if (time_samples.empty() ||
          (((t_first_transition - time_samples.back()) > epsilon) && ((t_sample - t_first_transition) > epsilon))) {
        time_samples.push_back(t_first_transition);
      }
      first_transition_added = true;
    }
    if (!second_transition_added && (t_sample > t_second_transition)) {
      if (((t_second_transition - time_samples.back()) > epsilon) && ((t_sample - t_second_transition) > epsilon)) {
        time_samples.push_back(t_second_transition);
      }
      second_transition_added = true;
    }
    time_samples.push_back(t_sample);
  }
  if (!second_transition_added) {
    time_samples.push_back(t_second_transition);
  }
  time_samples.push_back(trajectory.Duration());

  // sample the trajectory and solve the inverse kinematics
  Eigen::Isometry3d pose_sample;
  std::map<std::string, double> ik_solution_last, ik_solution, joint_velocity_last;
  ik_solution_last = initial_joint_position;
  for (const auto& item : ik_solution_last)
  {
    joint_velocity_last[item.first] = 0.0;
  }
  joint_trajectory.points.clear(); // clear previous run
  // set joint names
  joint_trajectory.joint_names.clear();
  for (const auto& start_joint : initial_joint_position)
  {
    joint_trajectory.joint_names.push_back(start_joint.first);
  }
  const std::vector<std::string> joint_names = joint_trajectory.joint_names;
  if (output_tcp_joints)
  {
    joint_trajectory.joint_names.push_back("tcp_lin");
    joint_trajectory.joint_names.push_back("tcp_rot");
  }

  bool success = true;
  max_scaling_factors.first = 1.0; // velocity
  max_scaling_factors.second = 1.0; // acceleration
  KDL::Frame frame_sample_last;
  KDL::RotationalInterpolation_SingleAxis rot_interpolation;
  double tcp_pos = 0.0;
  double tcp_rot = 0.0;
  double tcp_lin_vel_last = 0.0;
  double tcp_rot_vel_last = 0.0;
  double duration_last_sample = 0.0;
  for (std::vector<double>::const_iterator time_iter = time_samples.begin(); time_iter != time_samples.end();
       ++time_iter)
  {
    const auto frame_sample = trajectory.Pos(*time_iter);
    tf2::fromMsg(tf2::toMsg(frame_sample), pose_sample);

    if (!computePoseIK(scene, group_name, link_name, pose_sample, robot_model->getModelFrame(), ik_solution_last,
                       ik_solution, check_self_collision))
    {
      ROS_INFO("Failed to compute inverse kinematics solution for sampled "
               "Cartesian pose.");
      error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    // check the joint limits
    double duration_current_sample;
    if (time_iter != time_samples.begin())
    {
      duration_current_sample = *time_iter - *(time_iter - 1);
    }
    else {
      duration_current_sample = *time_iter;
    }
    if (time_iter == time_samples.begin()) {
      duration_last_sample = duration_current_sample;
    }

    // skip the first sample with zero time from start for limits checking
    std::pair<double, double> tmp_max_scaling_factors;
    if (time_iter != time_samples.begin() &&
        !verifySampleJointLimits(ik_solution_last, joint_velocity_last, ik_solution, duration_last_sample,
                                 duration_current_sample, joint_limits, tmp_max_scaling_factors))
    {
      ROS_DEBUG_STREAM("Inverse kinematics solution at "
                       << *time_iter << "s violates the joint velocity/acceleration/deceleration limits.");
      max_scaling_factors.first = std::min(max_scaling_factors.first, tmp_max_scaling_factors.first);
      max_scaling_factors.second = std::min(max_scaling_factors.second, tmp_max_scaling_factors.second);
      success = false;
      error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      if ((old_max_scaling_factors.first * tmp_max_scaling_factors.first < min_scaling_correction_factor) ||
          (old_max_scaling_factors.second * tmp_max_scaling_factors.second < min_scaling_correction_factor) ||
          strict_limits)
      {
        return false;
      }
    }

    // fill the point with joint values
    trajectory_msgs::JointTrajectoryPoint point;

    point.time_from_start = ros::Duration(*time_iter);
    for (const auto& joint_name : joint_names)
    {
      point.positions.push_back(ik_solution.at(joint_name));

      if (time_iter != time_samples.begin() && time_iter != time_samples.end() - 1)
      {
        double joint_velocity = (ik_solution.at(joint_name) - ik_solution_last.at(joint_name)) / duration_current_sample;
        point.velocities.push_back(joint_velocity);
        point.accelerations.push_back((joint_velocity - joint_velocity_last.at(joint_name)) /
                                      (duration_current_sample + duration_last_sample) * 2.0);
        joint_velocity_last[joint_name] = joint_velocity;
      }
      else
      {
        point.velocities.push_back(0.);
        point.accelerations.push_back(0.);
        joint_velocity_last[joint_name] = 0.;
      }
    }

    if (output_tcp_joints)
    {
      double tcp_lin_vel = 0.0;
      double tcp_rot_vel = 0.0;
      if (time_iter != time_samples.begin())
      {
        KDL::Frame diff = frame_sample * frame_sample_last.Inverse();
        double lin_distance = diff.p.Norm();
        tcp_pos += lin_distance;
        tcp_lin_vel = lin_distance / duration_current_sample;
        rot_interpolation.SetStartEnd(frame_sample_last.M, frame_sample.M);
        double rot_distance = rot_interpolation.Angle();
        tcp_rot += rot_distance;
        tcp_rot_vel = rot_distance / duration_current_sample;
      }
      point.positions.push_back(tcp_pos);  // tcp_lin
      point.positions.push_back(tcp_rot);  // tcp_rot
      if (time_iter != time_samples.begin() && time_iter != time_samples.end() - 1)
      {
        point.velocities.push_back(tcp_lin_vel);
        point.velocities.push_back(tcp_rot_vel);
        point.accelerations.push_back((tcp_lin_vel - tcp_lin_vel_last) /
                                      (duration_current_sample + duration_last_sample) * 2.0);
        point.accelerations.push_back((tcp_rot_vel - tcp_rot_vel_last) /
                                      (duration_current_sample + duration_last_sample) * 2.0);
        tcp_lin_vel_last = tcp_lin_vel;
        tcp_rot_vel_last = tcp_rot_vel;
      }
      else {
        point.velocities.push_back(0.0);
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
        point.accelerations.push_back(0.0);
        tcp_lin_vel_last = 0.0;
        tcp_rot_vel_last = 0.0;
      }

      frame_sample_last = frame_sample;
    }
    pose_sample_last = pose_sample;

    // update joint trajectory
    joint_trajectory.points.push_back(point);
    ik_solution_last = ik_solution;
    duration_last_sample = duration_current_sample;
  }

  if (success)
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    double duration_ms = (ros::Time::now() - generation_begin).toSec() * 1000;
    ROS_DEBUG_STREAM("Generate trajectory (N-Points: " << joint_trajectory.points.size() << ") took " << duration_ms
                                                       << " ms | " << duration_ms / joint_trajectory.points.size()
                                                       << " ms per Point");
  }
  return success;
}

bool pilz_industrial_motion_planner::generateJointTrajectory(
    const planning_scene::PlanningSceneConstPtr& scene,
    const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits,
    const pilz_industrial_motion_planner::CartesianTrajectory& trajectory, const std::string& group_name,
    const std::string& link_name, const std::map<std::string, double>& initial_joint_position,
    const std::map<std::string, double>& initial_joint_velocity, trajectory_msgs::JointTrajectory& joint_trajectory,
    moveit_msgs::MoveItErrorCodes& error_code, std::pair<double, double>& max_scaling_factors,
    bool check_self_collision, bool output_tcp_joints, bool strict_limits, double min_scaling_correction_factor)
{
  ROS_DEBUG("Generate joint trajectory from a Cartesian trajectory.");
  const auto old_max_scaling_factors = max_scaling_factors;

  const moveit::core::RobotModelConstPtr& robot_model = scene->getRobotModel();
  ros::Time generation_begin = ros::Time::now();

  std::map<std::string, double> ik_solution_last = initial_joint_position;
  std::map<std::string, double> joint_velocity_last = initial_joint_velocity;
  double duration_last = 0;
  double duration_current;
  joint_trajectory.joint_names.clear(); // clear previous run
  for (const auto& joint_position : ik_solution_last)
  {
    joint_trajectory.joint_names.push_back(joint_position.first);
  }
  std::vector<std::string> joint_names = joint_trajectory.joint_names;
  joint_names.erase(std::remove_if(joint_names.begin(), joint_names.end(),
                 [](const std::string &s){return (s == "tcp_lin") || (s == "tcp_rot");}), joint_names.end());

  bool success = true;
  max_scaling_factors.first = 1.0; // velocity
  max_scaling_factors.second = 1.0; // acceleration
  std::map<std::string, double> ik_solution;
  KDL::Frame frame_sample_last;
  KDL::RotationalInterpolation_SingleAxis rot_interpolation;
  double tcp_pos = 0.0;
  double tcp_rot = 0.0;
  double tcp_lin_vel_last = 0.0;
  double tcp_rot_vel_last = 0.0;
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    KDL::Frame frame_sample;
    tf2::fromMsg(trajectory.points.at(i).pose, frame_sample);
    // compute inverse kinematics
    if (!computePoseIK(scene, group_name, link_name, trajectory.points.at(i).pose, robot_model->getModelFrame(),
                       ik_solution_last, ik_solution, check_self_collision))
    {
      ROS_WARN("Failed to compute inverse kinematics solution for sampled "
                "Cartesian pose.");
      error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    // verify the joint limits
    if (i == 0)
    {
      duration_current = trajectory.points.front().time_from_start.toSec();
      duration_last = duration_current;
    }
    else
    {
      duration_current =
          trajectory.points.at(i).time_from_start.toSec() - trajectory.points.at(i - 1).time_from_start.toSec();
    }

    std::pair<double, double> tmp_max_scaling_factors;
    if (!verifySampleJointLimits(ik_solution_last, joint_velocity_last, ik_solution, duration_last, duration_current,
                                 joint_limits, tmp_max_scaling_factors))
    {
      // LCOV_EXCL_START since the same code was captured in a test in the other
      // overload generateJointTrajectory(...,
      // KDL::Trajectory, ...)
      // TODO: refactor to avoid code duplication.
      ROS_DEBUG_STREAM("Inverse kinematics solution of the " << i
                                                             << "th sample violates the joint "
                                                                "velocity/acceleration/deceleration limits.");
      max_scaling_factors.first = std::min(max_scaling_factors.first, tmp_max_scaling_factors.first);
      max_scaling_factors.second = std::min(max_scaling_factors.second, tmp_max_scaling_factors.second);
      success = false;
      error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      if ((old_max_scaling_factors.first * tmp_max_scaling_factors.first < min_scaling_correction_factor) ||
        (old_max_scaling_factors.second * tmp_max_scaling_factors.second < min_scaling_correction_factor) ||
        strict_limits)
      {
        return false;
      }
      // LCOV_EXCL_STOP
    }

    // compute the waypoint
    trajectory_msgs::JointTrajectoryPoint waypoint_joint;
    waypoint_joint.time_from_start = ros::Duration(trajectory.points.at(i).time_from_start);
    for (const auto& joint_name : joint_names)
    {
      waypoint_joint.positions.push_back(ik_solution.at(joint_name));
      double joint_velocity = (ik_solution.at(joint_name) - ik_solution_last.at(joint_name)) / duration_current;
      waypoint_joint.velocities.push_back(joint_velocity);
      waypoint_joint.accelerations.push_back((joint_velocity - joint_velocity_last.at(joint_name)) /
                                             (duration_current + duration_last) * 2.0);
      // update the joint velocity
      joint_velocity_last[joint_name] = joint_velocity;
    }

    if (output_tcp_joints)
    {
      double tcp_lin_vel = 0.0;
      double tcp_rot_vel = 0.0;
      if (i > 0) {
        KDL::Frame diff = frame_sample * frame_sample_last.Inverse();
        double lin_distance = diff.p.Norm();
        tcp_pos += lin_distance;
        tcp_lin_vel = lin_distance / duration_current;
        rot_interpolation.SetStartEnd(frame_sample_last.M, frame_sample.M);
        double rot_distance = rot_interpolation.Angle();
        tcp_rot += rot_distance;
        tcp_rot_vel = rot_distance / duration_current;
      }
      waypoint_joint.positions.push_back(tcp_pos);  // tcp_lin
      waypoint_joint.positions.push_back(tcp_rot);  // tcp_rot
      waypoint_joint.velocities.push_back(tcp_lin_vel);
      waypoint_joint.velocities.push_back(tcp_rot_vel);
      waypoint_joint.accelerations.push_back((tcp_lin_vel - tcp_lin_vel_last) / (duration_current + duration_last) * 2.0);
      waypoint_joint.accelerations.push_back((tcp_rot_vel - tcp_rot_vel_last) / (duration_current + duration_last) * 2.0);
      tcp_lin_vel_last = tcp_lin_vel;
      tcp_rot_vel_last = tcp_rot_vel;
      frame_sample_last = frame_sample;
    }

    // update joint trajectory
    joint_trajectory.points.push_back(waypoint_joint);
    ik_solution_last = ik_solution;
    duration_last = duration_current;
  }

  if (success)
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    double duration_ms = (ros::Time::now() - generation_begin).toSec() * 1000;
    ROS_DEBUG_STREAM("Generate trajectory (N-Points: " << joint_trajectory.points.size() << ") took " << duration_ms
                                                       << " ms | " << duration_ms / joint_trajectory.points.size()
                                                       << " ms per Point");
  }
  return true;
}

bool pilz_industrial_motion_planner::determineAndCheckSamplingTime(
    const robot_trajectory::RobotTrajectoryPtr& first_trajectory,
    const robot_trajectory::RobotTrajectoryPtr& second_trajectory, double epsilon, double& sampling_time)
{
  // The last sample is ignored because it is allowed to violate the sampling
  // time.
  std::size_t n1 = first_trajectory->getWayPointCount() - 1;
  std::size_t n2 = second_trajectory->getWayPointCount() - 1;
  if ((n1 < 2) && (n2 < 2))
  {
    ROS_ERROR_STREAM("Both trajectories do not have enough points to determine "
                     "sampling time.");
    return false;
  }

  if (n1 >= 2)
  {
    sampling_time = first_trajectory->getWayPointDurationFromPrevious(1);
  }
  else
  {
    sampling_time = second_trajectory->getWayPointDurationFromPrevious(1);
  }

  for (std::size_t i = 1; i < std::max(n1, n2); ++i)
  {
    if (i < n1)
    {
      if (fabs(sampling_time - first_trajectory->getWayPointDurationFromPrevious(i)) > epsilon)
      {
        ROS_ERROR_STREAM("First trajectory violates sampline time " << sampling_time << " between points " << (i - 1)
                                                                    << "and " << i << " (indices).");
        return false;
      }
    }

    if (i < n2)
    {
      if (fabs(sampling_time - second_trajectory->getWayPointDurationFromPrevious(i)) > epsilon)
      {
        ROS_ERROR_STREAM("Second trajectory violates sampline time " << sampling_time << " between points " << (i - 1)
                                                                     << "and " << i << " (indices).");
        return false;
      }
    }
  }

  return true;
}

bool pilz_industrial_motion_planner::isRobotStateEqual(const moveit::core::RobotState& state1,
                                                       const moveit::core::RobotState& state2,
                                                       const std::string& joint_group_name, double epsilon,
                                                       bool compare_velocity, bool compare_acceleration)
{
  Eigen::VectorXd joint_position_1, joint_position_2;
  std::string group_name = joint_group_name;
  if (group_name.rfind("_tcp") != std::string::npos) {
    group_name = group_name.substr(0, group_name.length() - 4);
  }

  state1.copyJointGroupPositions(group_name, joint_position_1);
  state2.copyJointGroupPositions(group_name, joint_position_2);

  if ((joint_position_1 - joint_position_2).norm() > epsilon)
  {
    ROS_DEBUG_STREAM("Joint positions of the two states are different. state1: " << joint_position_1
                                                                                 << " state2: " << joint_position_2);
    return false;
  }

  if (compare_velocity) {
    Eigen::VectorXd joint_velocity_1, joint_velocity_2;

    state1.copyJointGroupVelocities(group_name, joint_velocity_1);
    state2.copyJointGroupVelocities(group_name, joint_velocity_2);

    if ((joint_velocity_1 - joint_velocity_2).norm() > epsilon)
    {
      ROS_DEBUG_STREAM("Joint velocities of the two states are different. state1: " << joint_velocity_1
                                                                                    << " state2: " << joint_velocity_2);
      return false;
    }
  }

  if (compare_acceleration)
  {
    Eigen::VectorXd joint_acc_1, joint_acc_2;

    state1.copyJointGroupAccelerations(group_name, joint_acc_1);
    state2.copyJointGroupAccelerations(group_name, joint_acc_2);

    if ((joint_acc_1 - joint_acc_2).norm() > epsilon)
    {
      ROS_DEBUG_STREAM("Joint accelerations of the two states are different. state1: " << joint_acc_1
                                                                                       << " state2: " << joint_acc_2);
      return false;
    }
  }

  return true;
}

bool pilz_industrial_motion_planner::isRobotStateStationary(const moveit::core::RobotState& state,
                                                            const std::string& group, double EPSILON)
{
  Eigen::VectorXd joint_variable;
  state.copyJointGroupVelocities(group, joint_variable);
  if (joint_variable.norm() > EPSILON)
  {
    ROS_DEBUG("Joint velocities are not zero.");
    return false;
  }
  state.copyJointGroupAccelerations(group, joint_variable);
  if (joint_variable.norm() > EPSILON)
  {
    ROS_DEBUG("Joint accelerations are not zero.");
    return false;
  }
  return true;
}

bool pilz_industrial_motion_planner::linearSearchIntersectionPoint(const std::string& link_name,
                                                                   const Eigen::Vector3d& center_position,
                                                                   const double& r,
                                                                   const robot_trajectory::RobotTrajectoryPtr& traj,
                                                                   bool inverseOrder, std::size_t& index)
{
  ROS_DEBUG("Start linear search for intersection point.");

  const size_t waypoint_num = traj->getWayPointCount();

  if (inverseOrder)
  {
    for (size_t i = waypoint_num - 1; i > 0; --i)
    {
      if (intersectionFound(center_position, traj->getWayPointPtr(i)->getFrameTransform(link_name).translation(),
                            traj->getWayPointPtr(i - 1)->getFrameTransform(link_name).translation(), r))
      {
        index = i;
        return true;
      }
    }
  }
  else
  {
    for (size_t i = 0; i < waypoint_num - 1; ++i)
    {
      if (intersectionFound(center_position, traj->getWayPointPtr(i)->getFrameTransform(link_name).translation(),
                            traj->getWayPointPtr(i + 1)->getFrameTransform(link_name).translation(), r))
      {
        index = i;
        return true;
      }
    }
  }

  return false;
}

bool pilz_industrial_motion_planner::intersectionFound(const Eigen::Vector3d& p_center,
                                                       const Eigen::Vector3d& p_current, const Eigen::Vector3d& p_next,
                                                       const double& r)
{
  return ((p_current - p_center).norm() <= r) && ((p_next - p_center).norm() >= r);
}

bool pilz_industrial_motion_planner::isStateColliding(const bool test_for_self_collision,
                                                      const planning_scene::PlanningSceneConstPtr& scene,
                                                      robot_state::RobotState* rstate,
                                                      const robot_state::JointModelGroup* const group,
                                                      const double* const ik_solution)
{
  if (!test_for_self_collision)
  {
    return true;
  }

  rstate->setJointGroupPositions(group, ik_solution);
  rstate->update();
  collision_detection::CollisionRequest collision_req;
  collision_req.group_name = group->getName();
  collision_req.verbose = true;
  collision_detection::CollisionResult collision_res;
  scene->checkSelfCollision(collision_req, collision_res, *rstate);
  return !collision_res.collision;
}

void normalizeQuaternion(geometry_msgs::Quaternion& quat)
{
  tf2::Quaternion q;
  tf2::fromMsg(quat, q);
  quat = tf2::toMsg(q.normalize());
}

Eigen::Isometry3d getConstraintPose(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation,
                                    const geometry_msgs::Vector3& offset)
{
  Eigen::Quaterniond quat;
  tf2::fromMsg(orientation, quat);
  quat.normalize();
  Eigen::Vector3d v;
  tf2::fromMsg(position, v);

  Eigen::Isometry3d pose = Eigen::Translation3d(v) * quat;

  tf2::fromMsg(offset, v);
  pose.translation() -= quat * v;
  return pose;
}

Eigen::Isometry3d getConstraintPose(const moveit_msgs::Constraints& goal)
{
  return getConstraintPose(goal.position_constraints.front().constraint_region.primitive_poses.front().position,
                           goal.orientation_constraints.front().orientation,
                           goal.position_constraints.front().target_point_offset);
}
