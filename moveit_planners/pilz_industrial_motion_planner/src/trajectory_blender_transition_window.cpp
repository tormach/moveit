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

#include "pilz_industrial_motion_planner/trajectory_blender_transition_window.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <math.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/planning_interface/planning_interface.h>

namespace
{
// Motion of the blend link at a waypoint: position and rotation vector of the
// orientation relative to the waypoint, with their first and second time
// derivatives.
struct LinkMotion
{
  Eigen::Isometry3d pose;
  Eigen::Vector3d vel{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d acc{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d ang_vel{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d ang_acc{ Eigen::Vector3d::Zero() };
};

Eigen::Vector3d rotationVector(const Eigen::Matrix3d& rotation)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.angle() * angle_axis.axis();
}

// Velocity and acceleration at the middle one of three samples, taken h0
// before and h1 after it (exact for a parabola through the samples).
void finiteDifferences(const Eigen::Vector3d& x0, const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, double h0,
                       double h1, Eigen::Vector3d& vel, Eigen::Vector3d& acc)
{
  const Eigen::Vector3d slope0 = (x1 - x0) / h0;
  const Eigen::Vector3d slope1 = (x2 - x1) / h1;
  vel = (h1 * slope0 + h0 * slope1) / (h0 + h1);
  acc = 2.0 * (slope1 - slope0) / (h0 + h1);
}

// Motion of the link at waypoint index; the first and last waypoints of a
// trajectory are at rest.
LinkMotion linkMotionAt(const robot_trajectory::RobotTrajectory& traj, std::size_t index, const std::string& link_name)
{
  LinkMotion motion;
  motion.pose = traj.getWayPoint(index).getFrameTransform(link_name);
  if (index == 0 || index + 1 >= traj.getWayPointCount())
  {
    return motion;
  }
  const Eigen::Isometry3d pose0 = traj.getWayPoint(index - 1).getFrameTransform(link_name);
  const Eigen::Isometry3d pose2 = traj.getWayPoint(index + 1).getFrameTransform(link_name);
  const double h0 = traj.getWayPointDurationFromPrevious(index);
  const double h1 = traj.getWayPointDurationFromPrevious(index + 1);
  finiteDifferences(pose0.translation(), motion.pose.translation(), pose2.translation(), h0, h1, motion.vel, motion.acc);
  finiteDifferences(rotationVector(pose0.linear() * motion.pose.linear().transpose()), Eigen::Vector3d::Zero(),
                    rotationVector(pose2.linear() * motion.pose.linear().transpose()), h0, h1, motion.ang_vel,
                    motion.ang_acc);
  return motion;
}

// Time to cover the distance when the speed changes smoothly (as a cubic in
// time) from speed0 to speed1, with the accelerations acc0 and acc1 along the
// path at the ends; 0 if there is no motion.
double smoothTransitionTime(double distance, double speed0, double speed1, double acc0, double acc1)
{
  const double mean_speed = 0.5 * (speed0 + speed1);
  if (mean_speed <= std::numeric_limits<double>::epsilon())
  {
    return 0.0;
  }
  // distance = mean_speed * T + (acc0 - acc1) / 12 * T^2
  const double discriminant = mean_speed * mean_speed + (acc0 - acc1) / 3.0 * distance;
  if (discriminant < 0.0)
  {
    return distance / mean_speed;
  }
  return 2.0 * distance / (mean_speed + std::sqrt(discriminant));
}

// Quintic Hermite interpolation at time t of a duration from (x0, v0, a0) to
// (x1, v1, a1).
Eigen::Vector3d quinticHermite(const Eigen::Vector3d& x0, const Eigen::Vector3d& v0, const Eigen::Vector3d& a0,
                               const Eigen::Vector3d& x1, const Eigen::Vector3d& v1, const Eigen::Vector3d& a1,
                               double duration, double t)
{
  const double s = t / duration;
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  return (1 - 10 * s3 + 15 * s4 - 6 * s5) * x0 + duration * (s - 6 * s3 + 8 * s4 - 3 * s5) * v0 +
         duration * duration * 0.5 * (s2 - 3 * s3 + 3 * s4 - s5) * a0 + (10 * s3 - 15 * s4 + 6 * s5) * x1 +
         duration * (-4 * s3 + 7 * s4 - 3 * s5) * v1 + duration * duration * 0.5 * (s3 - 2 * s4 + s5) * a1;
}

// Speed along the direction of motion, and the acceleration along it.
void speedAndPathAcceleration(const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double& speed, double& path_acc)
{
  speed = vel.norm();
  path_acc = speed > std::numeric_limits<double>::epsilon() ? acc.dot(vel) / speed : 0.0;
}

// Length of the link's path (translation and rotation angle) between two
// waypoints.
void pathLength(const robot_trajectory::RobotTrajectory& traj, std::size_t first, std::size_t last,
                const std::string& link_name, double& distance, double& angle)
{
  Eigen::Isometry3d previous = traj.getWayPoint(first).getFrameTransform(link_name);
  for (std::size_t i = first + 1; i <= last; ++i)
  {
    const Eigen::Isometry3d pose = traj.getWayPoint(i).getFrameTransform(link_name);
    distance += (pose.translation() - previous.translation()).norm();
    angle += Eigen::AngleAxisd(pose.linear() * previous.linear().transpose()).angle();
    previous = pose;
  }
}
}  // namespace

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::blend(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
    pilz_industrial_motion_planner::TrajectoryBlendResponse& res)
{
  ROS_INFO("Start trajectory blending using transition window.");

  double sampling_time = 0.;
  if (!validateRequest(req, sampling_time, res.error_code))
  {
    ROS_ERROR("Trajectory blend request is not valid.");
    return false;
  }

  // search for intersection points of the two trajectories with the blending
  // sphere
  // intersection points belongs to blend trajectory after blending
  std::size_t first_intersection_index;
  std::size_t second_intersection_index;
  if (!searchIntersectionPoints(req, first_intersection_index, second_intersection_index))
  {
    ROS_ERROR("Blend radius to large.");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // blend the trajectories in Cartesian space
  pilz_industrial_motion_planner::CartesianTrajectory blend_trajectory_cartesian;
  double blend_time;
  if (!blendTrajectoryCartesian(req, first_intersection_index, second_intersection_index, sampling_time,
                                blend_trajectory_cartesian, blend_time))
  {
    ROS_ERROR("Blend curve leaves the blend sphere.");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // generate the blending trajectory in joint space
  std::map<std::string, double> initial_joint_position, initial_joint_velocity;
  for (const std::string& joint_name :
       req.first_trajectory->getFirstWayPointPtr()->getJointModelGroup(req.group_name)->getActiveJointModelNames())
  {
    initial_joint_position[joint_name] =
        req.first_trajectory->getWayPoint(first_intersection_index - 1).getVariablePosition(joint_name);
    initial_joint_velocity[joint_name] =
        req.first_trajectory->getWayPoint(first_intersection_index - 1).getVariableVelocity(joint_name);
  }
  trajectory_msgs::JointTrajectory blend_joint_trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  const bool output_tcp_joints = planning_parameters_->getOutputTcpJoints();
  const bool strict_limits = planning_parameters_->getStrictLimits();
  const double min_scaling_correction_factor = planning_parameters_->getMinScalingCorrectionFactor();
  bool succeeded = false;
  bool scaling_factor_corrected = false;
  double total_scaling_factor = 1.0;

  std::string group_name = req.group_name;
  if (group_name.rfind("_tcp") != std::string::npos) {
    group_name = group_name.substr(0, group_name.length() - 4);
  }
  while (!succeeded)
  {
    std::pair<double, double> max_scaling_factors { 1.0, 1.0 };
    if (!generateJointTrajectory(planning_scene, limits_.getJointLimitContainer(), blend_trajectory_cartesian,
                                 group_name, req.link_name, initial_joint_position, initial_joint_velocity,
                                 blend_joint_trajectory, error_code, max_scaling_factors, true, output_tcp_joints))
    {
      // LCOV_EXCL_START
      if (error_code.val != moveit_msgs::MoveItErrorCodes::PLANNING_FAILED)
      {
        break; // error not related to limit violation
      }
      else if (strict_limits) {
        break; // planning failed due to joint velocity/acceleration violation
      }

      double new_scaling_factor = std::min(max_scaling_factors.first, max_scaling_factors.second);
      // make sure scaling factor is lowered and does not diverge to 1.0
      new_scaling_factor = std::floor(new_scaling_factor / min_scaling_correction_factor) * min_scaling_correction_factor;
      total_scaling_factor *= new_scaling_factor;
      if (total_scaling_factor < min_scaling_correction_factor)
      {
        ROS_INFO_STREAM("Joint velocity or acceleration limit violated and below minimum scaling factor.");
        break; // would require scaling factor below threshold
      }

      ROS_DEBUG_STREAM("updating scaling factor " << total_scaling_factor);
      // limit violation, space out cartesian trajectory points
      scaleTrajectoryCartesian(blend_trajectory_cartesian, new_scaling_factor);
      scaling_factor_corrected = true;
      continue;
      // LCOV_EXCL_STOP
    }
    succeeded = true;
  }

  if (!succeeded) {
    ROS_INFO("Failed to generate joint trajectory for blending trajectory.");
    res.error_code.val = error_code.val;
    return false;
  }

  if (scaling_factor_corrected) {
    ROS_INFO_STREAM("Joint velocity or acceleration limit violated during blending. \nScaling factor has been corrected to: " <<
                    total_scaling_factor);
  }
  res.scaling_factor = total_scaling_factor;

  res.first_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(req.first_trajectory->getRobotModel(),
                                                                             req.first_trajectory->getGroup());
  res.blend_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(req.first_trajectory->getRobotModel(),
                                                                             req.first_trajectory->getGroup());
  res.second_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(req.first_trajectory->getRobotModel(),
                                                                              req.first_trajectory->getGroup());

  // set the three trajectories after blending in response
  // erase the points [first_intersection_index, back()] from the first
  // trajectory
  for (size_t i = 0; i < first_intersection_index; ++i)
  {
    res.first_trajectory->insertWayPoint(i, req.first_trajectory->getWayPoint(i),
                                         req.first_trajectory->getWayPointDurationFromPrevious(i));
  }

  // append the blend trajectory
  res.blend_trajectory->setRobotTrajectoryMsg(req.first_trajectory->getFirstWayPoint(), blend_joint_trajectory);
  // copy the points [second_intersection_index, len] from the second trajectory
  for (size_t i = second_intersection_index + 1; i < req.second_trajectory->getWayPointCount(); ++i)
  {
    res.second_trajectory->insertWayPoint(i - (second_intersection_index + 1), req.second_trajectory->getWayPoint(i),
                                          req.second_trajectory->getWayPointDurationFromPrevious(i));
  }

  // adjust the time from start: the second trajectory continues at the end of
  // the (possibly scaled) blend period
  res.second_trajectory->setWayPointDurationFromPrevious(
      0, blend_time / total_scaling_factor - blend_trajectory_cartesian.points.back().time_from_start.toSec());

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::validateRequest(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, double& sampling_time,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  ROS_DEBUG("Validate the trajectory blend request.");

  // check planning group
  if (!req.first_trajectory->getRobotModel()->hasJointModelGroup(req.group_name))
  {
    ROS_ERROR_STREAM("Unknown planning group: " << req.group_name);
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }

  // check link exists
  if (!req.first_trajectory->getRobotModel()->hasLinkModel(req.link_name) &&
      !req.first_trajectory->getLastWayPoint().hasAttachedBody(req.link_name))
  {
    ROS_ERROR_STREAM("Unknown link name: " << req.link_name);
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
    return false;
  }

  if (req.blend_radius <= 0)
  {
    ROS_ERROR("Blending radius must be positive");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // end position of the first trajectory and start position of second
  // trajectory must be the same
  if (!pilz_industrial_motion_planner::isRobotStateEqual(
          req.first_trajectory->getLastWayPoint(), req.second_trajectory->getFirstWayPoint(), req.group_name, EPSILON))
  {
    ROS_ERROR_STREAM("During blending the last point of the preceding and the first point of the succeding trajectory");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // same uniform sampling time
  if (!pilz_industrial_motion_planner::determineAndCheckSamplingTime(req.first_trajectory, req.second_trajectory,
                                                                     EPSILON, sampling_time))
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // end position of the first trajectory and start position of second
  // trajectory must have zero
  // velocities/accelerations
  if (!pilz_industrial_motion_planner::isRobotStateStationary(req.first_trajectory->getLastWayPoint(), req.group_name,
                                                              EPSILON) ||
      !pilz_industrial_motion_planner::isRobotStateStationary(req.second_trajectory->getFirstWayPoint(), req.group_name,
                                                              EPSILON))
  {
    ROS_ERROR("Intersection point of the blending trajectories has non-zero "
              "velocities/accelerations.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  return true;
}

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::blendTrajectoryCartesian(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, const std::size_t first_interse_index,
    const std::size_t second_interse_index, double sampling_time,
    pilz_industrial_motion_planner::CartesianTrajectory& trajectory, double& blend_time) const
{
  // other fields of the trajectory
  trajectory.group_name = req.group_name;
  trajectory.link_name = req.link_name;

  // the blend runs from the last point before the blend sphere to the first
  // point after it, which are not part of the blend
  const robot_trajectory::RobotTrajectory& first_traj = *req.first_trajectory;
  const robot_trajectory::RobotTrajectory& second_traj = *req.second_trajectory;
  const std::size_t start_index = first_interse_index - 1;
  const std::size_t end_index = second_interse_index + 1;
  const LinkMotion start = linkMotionAt(first_traj, start_index, req.link_name);
  const LinkMotion end = linkMotionAt(second_traj, end_index, req.link_name);

  // blend duration: the time to cover the path the two trajectories take
  // through the blend sphere, with the speed changing smoothly from the
  // speed entering the blend to the speed leaving it. As in the LIN/CIRC
  // planners, the rotation counts only where it is the longer motion (angle
  // times max_trans_vel / max_rot_vel): a duration set by a slow rotation
  // would stretch the translation curve out of the sphere.
  double distance = 0.0;
  double angle = 0.0;
  pathLength(first_traj, start_index, first_traj.getWayPointCount() - 1, req.link_name, distance, angle);
  pathLength(second_traj, 0, end_index, req.link_name, distance, angle);
  double speed0, speed1, path_acc0, path_acc1;
  speedAndPathAcceleration(start.vel, start.acc, speed0, path_acc0);
  speedAndPathAcceleration(end.vel, end.acc, speed1, path_acc1);
  const double translation_time = smoothTransitionTime(distance, speed0, speed1, path_acc0, path_acc1);
  speedAndPathAcceleration(start.ang_vel, start.ang_acc, speed0, path_acc0);
  speedAndPathAcceleration(end.ang_vel, end.ang_acc, speed1, path_acc1);
  const double rotation_time = smoothTransitionTime(angle, speed0, speed1, path_acc0, path_acc1);
  const double rotation_radius = limits_.getCartesianLimits().getMaxTranslationalVelocity() /
                                 limits_.getCartesianLimits().getMaxRotationalVelocity();
  blend_time = (distance >= angle * rotation_radius && translation_time > 0.0) ? translation_time : rotation_time;
  if (blend_time <= 0.0)
  {
    // no motion into or out of the blend: keep the time of the trajectories
    blend_time = first_traj.getDuration() - first_traj.getWayPointDurationFromStart(start_index) +
                 second_traj.getWayPointDurationFromStart(end_index);
  }

  // the orientation is interpolated as a rotation vector relative to the
  // start orientation
  const Eigen::Vector3d end_rotation = rotationVector(end.pose.linear() * start.pose.linear().transpose());

  // the blend curve must stay in the blend sphere (allowing for the start and
  // end points, which are just outside it)
  const Eigen::Vector3d center = first_traj.getLastWayPoint().getFrameTransform(req.link_name).translation();
  const double max_distance =
      1.1 * std::max((start.pose.translation() - center).norm(), (end.pose.translation() - center).norm());

  // the samples are evenly spaced in time between the start (t = 0) and the
  // end (t = blend_time)
  const std::size_t blend_sample_num = std::max<std::size_t>(2, std::lround(blend_time / sampling_time));
  pilz_industrial_motion_planner::CartesianTrajectoryPoint waypoint;

  // Pose on blending trajectory
  Eigen::Isometry3d blend_sample_pose = Eigen::Isometry3d::Identity();
  for (std::size_t i = 1; i < blend_sample_num; ++i)
  {
    const double t = blend_time * i / blend_sample_num;

    blend_sample_pose.translation() = quinticHermite(start.pose.translation(), start.vel, start.acc,
                                                     end.pose.translation(), end.vel, end.acc, blend_time, t);
    if ((blend_sample_pose.translation() - center).norm() > max_distance)
    {
      return false;
    }

    const Eigen::Vector3d rotation = quinticHermite(Eigen::Vector3d::Zero(), start.ang_vel, start.ang_acc, end_rotation,
                                                    end.ang_vel, end.ang_acc, blend_time, t);
    const double rotation_angle = rotation.norm();
    const Eigen::Matrix3d relative_rotation =
        rotation_angle > std::numeric_limits<double>::epsilon() ?
            Eigen::AngleAxisd(rotation_angle, rotation / rotation_angle).toRotationMatrix() :
            Eigen::Matrix3d::Identity();
    blend_sample_pose.linear() = relative_rotation * start.pose.linear();

    // push to the trajectory
    waypoint.pose = tf2::toMsg(blend_sample_pose);
    waypoint.time_from_start = ros::Duration(t);
    trajectory.points.push_back(waypoint);
  }
  return true;
}

void pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::scaleTrajectoryCartesian(pilz_industrial_motion_planner::CartesianTrajectory& trajectory, double scale_factor) const
{
  for (auto &point: trajectory.points) {
    point.time_from_start *= 1.0 / scale_factor;
  }
}

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::searchIntersectionPoints(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, std::size_t& first_interse_index,
    std::size_t& second_interse_index) const
{
  ROS_INFO("Search for start and end point of blending trajectory.");

  // compute the position of the center of the blend sphere
  // (last point of the first trajectory, first point of the second trajectory)
  Eigen::Isometry3d circ_pose = req.first_trajectory->getLastWayPoint().getFrameTransform(req.link_name);

  // Searh for intersection points according to distance
  if (!linearSearchIntersectionPoint(req.link_name, circ_pose.translation(), req.blend_radius, req.first_trajectory,
                                     true, first_interse_index))
  {
    ROS_ERROR_STREAM("Intersection point of first trajectory not found.");
    return false;
  }
  ROS_INFO_STREAM("Intersection point of first trajectory found, index: " << first_interse_index);

  if (!linearSearchIntersectionPoint(req.link_name, circ_pose.translation(), req.blend_radius, req.second_trajectory,
                                     false, second_interse_index))
  {
    ROS_ERROR_STREAM("Intersection point of second trajectory not found.");
    return false;
  }

  ROS_INFO_STREAM("Intersection point of second trajectory found, index: " << second_interse_index);
  return true;
}
