#ifndef NORLAB_ICP_MAPPER_IMUMEASUREMENT_H
#define NORLAB_ICP_MAPPER_IMUMEASUREMENT_H

#include <chrono>
#include <Eigen/Core>

struct ImuMeasurement
{
    std::chrono::time_point<std::chrono::steady_clock> timeStamp;
    Eigen::Matrix<float, 3, 1> angularVelocity;
    Eigen::Matrix<float, 3, 1> linearAcceleration;
};

#endif
