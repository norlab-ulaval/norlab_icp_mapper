#include "Trajectory.h"
#include <pointmatcher/PointMatcher.h>

Trajectory::Trajectory(int dimension):
        dimension(dimension)
{
}

void Trajectory::addPose(Eigen::MatrixXf pose, std::chrono::time_point<std::chrono::steady_clock> timeStamp)
{
    poses.push_back(pose);
    timeStamps.push_back(timeStamp);
}

void Trajectory::save(std::string filename) const
{
    PointMatcher<float>::DataPoints::Labels featureLabels;
    featureLabels.push_back(PointMatcher<float>::DataPoints::Label("x", 1));
    featureLabels.push_back(PointMatcher<float>::DataPoints::Label("y", 1));
    if(dimension == 3)
    {
        featureLabels.push_back(PointMatcher<float>::DataPoints::Label("z", 1));
    }
    Eigen::MatrixXf features(dimension, poses.size());

    PointMatcher<float>::DataPoints::Labels descriptorLabels;
    descriptorLabels.push_back(PointMatcher<float>::DataPoints::Label("orientation_x", dimension));
    descriptorLabels.push_back(PointMatcher<float>::DataPoints::Label("orientation_y", dimension));
    if(dimension == 3)
    {
        descriptorLabels.push_back(PointMatcher<float>::DataPoints::Label("orientation_z", dimension));
    }
    Eigen::MatrixXf descriptors(dimension * dimension, poses.size());

    PointMatcher<float>::DataPoints::Labels timeLabels;
    timeLabels.push_back(PointMatcher<float>::DataPoints::Label("t", 1));
    Eigen::Matrix<std::int64_t, 1, Eigen::Dynamic> times(1, poses.size());

    for(size_t i = 0; i < poses.size(); ++i)
    {
        features.col(i) = poses[i].topRightCorner(dimension, 1);
        descriptors.block(0, i, dimension, 1) = poses[i].block(0, 0, dimension, 1);
        descriptors.block(dimension, i, dimension, 1) = poses[i].block(0, 1, dimension, 1);
        if(dimension == 3)
        {
            descriptors.block(2 * dimension, i, dimension, 1) = poses[i].block(0, 2, dimension, 1);
        }
        times(0, i) = timeStamps[i].time_since_epoch().count();
    }

    PointMatcher<float>::DataPoints trajectory(features, featureLabels, descriptors, descriptorLabels, times, timeLabels);
    trajectory.save(filename);
}

void Trajectory::clear()
{
    poses.clear();
    timeStamps.clear();
}
