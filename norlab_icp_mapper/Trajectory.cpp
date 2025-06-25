#include "Trajectory.h"
#include <pointmatcher/PointMatcher.h>
#include <filesystem>
#include<fstream>

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
    using PM = PointMatcher<float>;
    std::filesystem::path outputPath = filename;
    if (outputPath.extension() == ".tum")
    {
        // tum format is defined as timestamp x y z q_x q_y q_z q_w
        std::ofstream file(outputPath);
        for (int i=0; i<this->poses.size(); ++i) {
            PM::TransformationParameters pose = poses.at(i);

            auto timestamp = timeStamps.at(i);

            // export timeestamp
            file << timestamp.time_since_epoch().count() << " ";
            // export x y z
            file << pose(0, 3) << " " << pose(1, 3) << " " << pose(2, 3) << " ";
            // export rotation as a quaternion
            Eigen::Quaternionf q(pose.block<3,3>(0,0));
            file << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        file.close();
    }
    else
    {
        PointMatcher<float>::DataPoints::Labels featureLabels;
        featureLabels.push_back(PointMatcher<float>::DataPoints::Label("x", 1));
        featureLabels.push_back(PointMatcher<float>::DataPoints::Label("y", 1));
        if(dimension == 3)
        {
            featureLabels.push_back(PointMatcher<float>::DataPoints::Label("z", 1));
        }
        featureLabels.push_back(PointMatcher<float>::DataPoints::Label("pad", 1));
        Eigen::MatrixXf features(dimension, poses.size());

        PointMatcher<float>::DataPoints::Labels descriptorLabels;
        descriptorLabels.push_back(PointMatcher<float>::DataPoints::Label("orientationX", dimension));
        descriptorLabels.push_back(PointMatcher<float>::DataPoints::Label("orientationY", dimension));
        if(dimension == 3)
        {
            descriptorLabels.push_back(PointMatcher<float>::DataPoints::Label("orientationZ", dimension));
        }
        Eigen::MatrixXf descriptors(dimension * dimension, poses.size());

        PointMatcher<float>::DataPoints::Labels timeLabels;
        timeLabels.push_back(PointMatcher<float>::DataPoints::Label("time", 1));
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
}

void Trajectory::clear()
{
    poses.clear();
    timeStamps.clear();
}
