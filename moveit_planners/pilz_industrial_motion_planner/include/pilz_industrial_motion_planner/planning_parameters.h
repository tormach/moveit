//
// Created by alexander on 2/24/22.
//

#pragma once

#include <ros/ros.h>

namespace pilz_industrial_motion_planner
{

class PlanningParameters
{
public:
  PlanningParameters(const ros::NodeHandle& nh);

  double getSamplingTime() const;
  bool getOutputTcpJoints() const;
  bool getTrimOnFailure() const;
  bool getStrictLimits() const;
  double getMinScalingCorrectionFactor() const;

private:
  ros::NodeHandle nh_;
};

}
