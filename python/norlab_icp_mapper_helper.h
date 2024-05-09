//
// Created by mbo on 19/05/22.
//

#ifndef NORLAB_ICP_MAPPER_NORLAB_ICP_MATCHER_HELPER_H
#define NORLAB_ICP_MAPPER_NORLAB_ICP_MATCHER_HELPER_H

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include "pointmatcher/PointMatcher.h"

#include "norlab_icp_mapper/Map.h"
#include "norlab_icp_mapper/CellManager.h"
#include "norlab_icp_mapper/Mapper.h"
#include "norlab_icp_mapper/Trajectory.h"

namespace py = pybind11;

// PointMatcher aliases
using PM = PointMatcher<float>;
using DataPoints = PM::DataPoints;

#endif //NORLAB_ICP_MAPPER_NORLAB_ICP_MATCHER_HELPER_H
