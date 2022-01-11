//
// Created by alexander on 2/24/22.
//
#include "pilz_industrial_motion_planner/planning_parameters.h"

static const std::string PARAMS_PLANNING_PARAMS_NS = "planning_parameters";

static const std::string PARAMS_SAMPLING_TIME = "sampling_time";
static const std::string PARAMS_OUTPUT_TCP_JOINTS = "output_tcp_joints";
static const std::string PARAMS_TRIM_ON_FAILURE = "trim_on_failure";
static const std::string PARAMS_STRICT_LIMITS = "strict_limits";
static const std::string PARAMS_MIN_SCALING_CORRECTION_FACTOR = "min_scaling_correction_factor";

pilz_industrial_motion_planner::PlanningParameters::PlanningParameters(const ros::NodeHandle& nh):
    nh_(nh)
{
}

double pilz_industrial_motion_planner::PlanningParameters::getSamplingTime() const
{
  double sampling_time = 0.1;
  if (!nh_.getParam(PARAMS_PLANNING_PARAMS_NS + "/" + PARAMS_SAMPLING_TIME, sampling_time))
  {
    ROS_WARN_ONCE("Planning parameter sampling_time not set, using default %f instead.", sampling_time);
  }
  return sampling_time;
}

bool pilz_industrial_motion_planner::PlanningParameters::getOutputTcpJoints() const
{
  bool output_tcp_joints = false;
  if (!nh_.getParam(PARAMS_PLANNING_PARAMS_NS + "/" + PARAMS_OUTPUT_TCP_JOINTS, output_tcp_joints))
  {
    ROS_WARN_ONCE("Planning parameter output_tcp_joints not set, using default %u instead.", output_tcp_joints);
  }
  return output_tcp_joints;
}

bool pilz_industrial_motion_planner::PlanningParameters::getTrimOnFailure() const
{
  bool trim_on_failure = false;
  if (!nh_.getParam(PARAMS_PLANNING_PARAMS_NS + "/" + PARAMS_TRIM_ON_FAILURE, trim_on_failure))
  {
    ROS_WARN_ONCE("Planning parameter trim_on_failure not set, using default %u instead", trim_on_failure);
  }
  return trim_on_failure;
}
bool pilz_industrial_motion_planner::PlanningParameters::getStrictLimits() const
{
  bool strict_limits = true;
  if (!nh_.getParam(PARAMS_PLANNING_PARAMS_NS + "/" + PARAMS_STRICT_LIMITS, strict_limits))
  {
    ROS_WARN_ONCE("Planning parameter strict_limits not set, using default %u instead", strict_limits);
  }
  return strict_limits;
}
double pilz_industrial_motion_planner::PlanningParameters::getMinScalingCorrectionFactor() const
{
  double min_scaling_correction_factor = 0.01;
  if (!nh_.getParam(PARAMS_PLANNING_PARAMS_NS + "/" + PARAMS_MIN_SCALING_CORRECTION_FACTOR, min_scaling_correction_factor))
  {
    ROS_WARN_ONCE("Planning parameter min_scaling_correction_factor not set, using default %f instead.", min_scaling_correction_factor);
  }
  return min_scaling_correction_factor;
}
