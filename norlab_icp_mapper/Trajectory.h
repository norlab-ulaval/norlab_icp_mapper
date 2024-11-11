#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>
#include <pointmatcher/PointMatcher.h>
#include <chrono>
#include <vector>


class Trajectory
{
public:
    typedef PointMatcher<float> PM;
    Trajectory(int dimension);
    void addPose(Eigen::MatrixXf pose, std::chrono::time_point<std::chrono::steady_clock> timeStamp, PM::TransformationParameters correction);
    void save(std::string filename) const;
    void clear();

private:
    int dimension;
    std::vector<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> poses;
    std::vector<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> icpCorrections;
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> timeStamps;
};

#endif
