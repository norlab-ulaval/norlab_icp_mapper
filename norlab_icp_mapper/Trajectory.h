#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>
#include <chrono>
#include <vector>

class Trajectory
{
public:
    Trajectory(int dimension);
    void addPose(Eigen::MatrixXf pose, std::chrono::time_point<std::chrono::steady_clock> timeStamp);
    void save(std::string filename) const;
    void clear();

private:
    int dimension;
    std::vector<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> poses;
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> timeStamps;
};

#endif
