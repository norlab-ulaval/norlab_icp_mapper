#ifndef NORLAB_ICP_MAPPER_STAMPEDSTATE_H
#define NORLAB_ICP_MAPPER_STAMPEDSTATE_H

#include <chrono>
#include <Eigen/Core>

struct StampedState
{
    std::chrono::time_point<std::chrono::steady_clock> timeStamp;
    Eigen::Matrix<float, 4, 4> pose;
    Eigen::Matrix<float, 3, 1> velocity;
};

#endif
