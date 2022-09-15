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

#include "pilz_industrial_motion_planner/plan_components_builder.h"

#include <cassert>

#include "pilz_industrial_motion_planner/tip_frame_getter.h"

namespace pilz_industrial_motion_planner
{
std::vector<robot_trajectory::RobotTrajectoryPtr> PlanComponentsBuilder::build() const
{
  std::vector<robot_trajectory::RobotTrajectoryPtr> res_vec{ traj_cont_ };
  if (traj_tail_)
  {
    assert(!res_vec.empty());
    appendWithStrictTimeIncrease(*(res_vec.back()), *traj_tail_);
  }
  return res_vec;
}

void PlanComponentsBuilder::appendWithStrictTimeIncrease(robot_trajectory::RobotTrajectory& result,
                                                         const robot_trajectory::RobotTrajectory& source)
{
  std::string group_name = result.getGroupName();
  double tcp_lin_offset = 0.0;
  double tcp_rot_offset = 0.0;
  bool is_tcp_group = false;
  if (group_name.rfind("_tcp") != std::string::npos) {
    group_name = group_name.substr(0, group_name.size()-4);
    is_tcp_group = true;
    if (!result.empty())
    {
      tcp_lin_offset = *result.getLastWayPoint().getJointPositions("tcp_lin");
      tcp_rot_offset = *result.getLastWayPoint().getJointPositions("tcp_rot");
    }
  }

  size_t start_index = 1;
  if (result.empty() ||
      !pilz_industrial_motion_planner::isRobotStateEqual(result.getLastWayPoint(), source.getFirstWayPoint(),
                                                         group_name, ROBOT_STATE_EQUALITY_EPSILON))
  {
    start_index = 0;
  }

  for (size_t i = start_index; i < source.getWayPointCount(); ++i)
  {
    if (is_tcp_group) {
      moveit::core::RobotState new_waypoint = source.getWayPoint(i);
      const double tcp_lin = new_waypoint.getVariablePosition("tcp_lin") + tcp_lin_offset;
      const double tcp_rot = new_waypoint.getVariablePosition("tcp_rot") + tcp_rot_offset;
      new_waypoint.setVariablePosition("tcp_lin", tcp_lin);
      new_waypoint.setVariablePosition("tcp_rot", tcp_rot);
      result.addSuffixWayPoint(new_waypoint, source.getWayPointDurationFromPrevious(i));
    }
    else {
      result.addSuffixWayPoint(source.getWayPoint(i), source.getWayPointDurationFromPrevious(i));
    }
  }
}

void PlanComponentsBuilder::blend(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const robot_trajectory::RobotTrajectoryPtr& other, const double blend_radius)
{
  if (!blender_)
  {
    throw NoBlenderSetException("No blender set");
  }

  assert(other->getGroupName() == traj_tail_->getGroupName());

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_request;

  blend_request.first_trajectory = traj_tail_;
  blend_request.second_trajectory = other;
  blend_request.blend_radius = blend_radius;
  blend_request.group_name = traj_tail_->getGroupName();
  std::string group_name = blend_request.group_name;
  if (group_name.rfind("_tcp") != std::string::npos) {
    group_name = group_name.substr(0, group_name.length() - 4);
  }
  blend_request.link_name = getSolverTipFrame(model_->getJointModelGroup(group_name));

  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_response;
  if (!blender_->blend(planning_scene, blend_request, blend_response))
  {
    throw BlendingFailedException("Blending failed");
  }

  // Append the new trajectory elements
  appendWithStrictTimeIncrease(*(traj_cont_.back()), *blend_response.first_trajectory);
  appendWithStrictTimeIncrease(*(traj_cont_.back()), *blend_response.blend_trajectory);
  // Store the last new trajectory element for future processing
  traj_tail_ = blend_response.second_trajectory;  // first for next blending segment
}

void PlanComponentsBuilder::append(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                   const robot_trajectory::RobotTrajectoryPtr& other, const double blend_radius)
{
  if (!model_)
  {
    throw NoRobotModelSetException("No robot model set");
  }

  if (!traj_tail_)
  {
    traj_tail_ = other;
    // Reserve space in container for new trajectory
    traj_cont_.emplace_back(new robot_trajectory::RobotTrajectory(model_, other->getGroupName()));
    return;
  }

  // Create new trajectory for every group change
  if (other->getGroupName() != traj_tail_->getGroupName())
  {
    appendWithStrictTimeIncrease(*(traj_cont_.back()), *traj_tail_);
    traj_tail_ = other;
    // Create new container element
    traj_cont_.emplace_back(new robot_trajectory::RobotTrajectory(model_, other->getGroupName()));
    return;
  }

  // No blending
  if (blend_radius <= 0.0)
  {
    appendWithStrictTimeIncrease(*(traj_cont_.back()), *traj_tail_);
    traj_tail_ = other;
    return;
  }

  blend(planning_scene, other, blend_radius);
}

}  // namespace pilz_industrial_motion_planner
