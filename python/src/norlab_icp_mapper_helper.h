#ifndef NORLAB_ICP_MAPPER_NORLAB_ICP_MATCHER_HELPER_H
#define NORLAB_ICP_MAPPER_NORLAB_ICP_MATCHER_HELPER_H

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include "pointmatcher/PointMatcher.h"

namespace py = pybind11;

// PointMatcher aliases
using PM = PointMatcher<float>;
using DataPoints = PM::DataPoints;

#endif //NORLAB_ICP_MAPPER_NORLAB_ICP_MATCHER_HELPER_H
