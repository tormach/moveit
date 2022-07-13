//
// Created by alexander on 7/11/22.
//

#pragma once

#include "kdl/trajectory_segment.hpp"
#include "pilz_industrial_motion_planner/velocity_profile.h"

namespace pilz_industrial_motion_planner {
  class Trajectory_Segment : public KDL::Trajectory_Segment
  {
    VelocityProfile *motprof;
  public:
    Trajectory_Segment(KDL::Path* _geom, VelocityProfile* _motprof, bool _aggregate=true):
    KDL::Trajectory_Segment(_geom, _motprof, _aggregate) {
      motprof = _motprof;
    }

    const VelocityProfile* getVelocityProfile() const {
      return motprof;
    }
  };
} // namespace pilz_industrial_motion_planner
