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

#pragma once

#include <moveit/planning_interface/planning_interface.h>

#include "pilz_industrial_motion_planner/cartesian_trajectory.h"
#include "pilz_industrial_motion_planner/cartesian_trajectory_point.h"
#include "pilz_industrial_motion_planner/trajectory_blend_request.h"
#include "pilz_industrial_motion_planner/trajectory_blender.h"
#include "pilz_industrial_motion_planner/trajectory_functions.h"

namespace pilz_industrial_motion_planner
{
/**
 * @brief Trajectory blender implementing transition window algorithm
 *
 * See doc/MotionBlendAlgorithmDescription.pdf for a mathematical description of
 * the algorithmn.
 */
class TrajectoryBlenderTransitionWindow : public TrajectoryBlender
{
public:
  TrajectoryBlenderTransitionWindow(const LimitsContainer& planner_limits,
                                    std::shared_ptr<pilz_industrial_motion_planner::PlanningParameters> planning_parameters)
    : TrajectoryBlender::TrajectoryBlender(planner_limits, std::move(planning_parameters))
  {
  }

  ~TrajectoryBlenderTransitionWindow() override
  {
  }

  /**
   * @brief Blend two trajectories using transition window. The trajectories
   * have to be equally and uniformly
   * discretized.
   * @param planning_scene: The scene planning is occurring in.
   * @param req: following fields need to be filled for a valid request:
   *    - group_name : name of the planning group
   *    - link_name : name of the target link
   *    - first_trajectory: Joint trajectory stops at end point.
   *                        The last point must be the same as the first point
   * of the second trajectory.
   *    - second trajectory: Joint trajectory stops at end point.
   *                         The first point must be the same as the last point
   * of the first trajectory.
   *    - blend_radius: The blend radius determines a sphere with the
   * intersection point of the two trajectories
   *                    as the center. Trajectory blending happens inside of
   * this sphere.
   * @param res: following fields are returned as response by the blend
   * algorithm
   *    - group_name : name of the planning group
   *    - first_trajectory: Part of the first original trajectory which is
   * outside of the blend sphere.
   *    - blend_trajectory: Joint trajectory connecting the first and second
   * trajectories without stop.
   *                        The first waypoint has non-zero time from start.
   *    - second trajectory: Part of the second original trajectory which is
   * outside of the blend sphere.
   *                         The first waypoint has non-zero time from start.
   * error_code: information of failed blend
   * @return true if succeed
   */
  bool blend(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
             pilz_industrial_motion_planner::TrajectoryBlendResponse& res) override;

private:
  /**
   * @brief validate trajectory blend request
   * @param req
   * @param sampling_time: get the same sampling time of the two input
   * trajectories
   * @param error_code
   * @return
   */
  bool validateRequest(const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, double& sampling_time,
                       moveit_msgs::MoveItErrorCodes& error_code) const;
  /**
   * @brief searchBlendPoint
   * @param req: trajectory blend request
   * @param first_interse_index: index of the first point of the first
   * trajectory that is inside the blend sphere
   * @param second_interse_index: index of the last point of the second
   * trajectory that is still inside the blend sphere
   */
  bool searchIntersectionPoints(const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
                                std::size_t& first_interse_index, std::size_t& second_interse_index) const;

  /**
   * @brief blend two trajectories in Cartesian space, result in a
   * MultiDOFJointTrajectory which consists
   * of a list of transforms for the blend phase.
   * The blend is a quintic Hermite curve in position and in orientation
   * (rotation vector) from the last point of the first trajectory before the
   * blend sphere to the first point of the second trajectory after it, with
   * the velocities and accelerations of the two trajectories at these points.
   * Its duration is chosen so the speed changes smoothly from the speed
   * entering the blend to the speed leaving it, over the path length the two
   * trajectories would have covered inside the sphere, so the blend does not
   * overshoot the speed of the trajectories next to it, and slows down in
   * sharp corners. As in the LIN/CIRC planners, the rotation sets the
   * duration only where it is the longer motion.
   * @param req
   * @param first_interse_index
   * @param second_interse_index
   * @param sampling_time: approximate time between the blend samples
   * @param trajectory: the resulting blend trajectory inside the blending
   * sphere; the first point after second_interse_index on the second
   * trajectory follows its last sample at blend_time
   * @param blend_time: duration of the blend, from the last point before the
   * sphere to the first point after it
   * @return false if the blend curve leaves the blend sphere (by more than
   * its start and end points do)
   */
  bool blendTrajectoryCartesian(const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
                                const std::size_t first_interse_index, const std::size_t second_interse_index,
                                double sampling_time, pilz_industrial_motion_planner::CartesianTrajectory& trajectory,
                                double& blend_time) const;

  void scaleTrajectoryCartesian(pilz_industrial_motion_planner::CartesianTrajectory& trajectory, double scale_factor) const;

private:  // static members
  // Constant to check for equality of values.
  static constexpr double EPSILON = 1e-4;
};

}  // namespace pilz_industrial_motion_planner
