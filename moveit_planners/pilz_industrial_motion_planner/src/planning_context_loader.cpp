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

#include "pilz_industrial_motion_planner/planning_context_loader.h"
#include <ros/ros.h>

#include <utility>

pilz_industrial_motion_planner::PlanningContextLoader::PlanningContextLoader() :
  limits_set_(false),
  model_set_(false),
  error_details_set_(false),
  planning_parameters_set_(false)
{
}

pilz_industrial_motion_planner::PlanningContextLoader::~PlanningContextLoader()
{
}

bool pilz_industrial_motion_planner::PlanningContextLoader::setModel(const moveit::core::RobotModelConstPtr& model)
{
  model_ = model;
  model_set_ = true;
  return true;
}

bool pilz_industrial_motion_planner::PlanningContextLoader::setLimits(
    const pilz_industrial_motion_planner::LimitsContainer& limits)
{
  limits_ = limits;
  limits_set_ = true;
  return true;
}

bool pilz_industrial_motion_planner::PlanningContextLoader::setErrorDetails(
    std::shared_ptr<pilz_industrial_motion_planner::ErrorDetailsContainer> error_details)
{
  error_details_ = std::move(error_details);
  error_details_set_ = true;
  return true;
}

bool pilz_industrial_motion_planner::PlanningContextLoader::setPlanningParameters(
    std::shared_ptr<PlanningParameters> planning_parameters)
{
  planning_parameters_ = std::move(planning_parameters);
  planning_parameters_set_ = true;
  return true;
}

std::string pilz_industrial_motion_planner::PlanningContextLoader::getAlgorithm() const
{
  return alg_;
}
