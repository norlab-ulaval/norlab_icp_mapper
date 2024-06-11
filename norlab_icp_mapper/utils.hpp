#include <pointmatcher/PointMatcher.h>
#include "StampedState.h"
#include <chrono>

typedef PointMatcher<float> PM;

PM::DataPoints deskew(const PM::DataPoints& skewedCloud, const std::chrono::time_point<std::chrono::steady_clock>& cloudStamp,
                      const std::vector<StampedState>& trajectory)
{
    PM::DataPoints::ConstView pointStamps = skewedCloud.getDescriptorViewByName("t");
    PM::DataPoints deskewedCloud(skewedCloud);
    for(unsigned int i = 0; i < skewedCloud.getNbPoints(); ++i)
    {
        int previousPoseIndex = 0;
        while(previousPoseIndex + 2 < trajectory.size() && trajectory[previousPoseIndex + 1].timeStamp <= cloudStamp + std::chrono::nanoseconds((long)pointStamps(0, i)))
        {
            previousPoseIndex += 1;
        }

        std::chrono::time_point<std::chrono::steady_clock> previousStamp = trajectory[previousPoseIndex].timeStamp;
        std::chrono::time_point<std::chrono::steady_clock> nextStamp = trajectory[previousPoseIndex + 1].timeStamp;
        Eigen::Quaternion<float> previousOrientation(trajectory[previousPoseIndex].pose.topLeftCorner<3, 3>());
        Eigen::Quaternion<float> nextOrientation(trajectory[previousPoseIndex + 1].pose.topLeftCorner<3, 3>());
        Eigen::Matrix<float, 3, 1> previousPosition = trajectory[previousPoseIndex].pose.topRightCorner<3, 1>();
        Eigen::Matrix<float, 3, 1> nextPosition = trajectory[previousPoseIndex + 1].pose.topRightCorner<3, 1>();

        float ratio = ((float)((cloudStamp + std::chrono::nanoseconds((long)pointStamps(0, i))) - previousStamp).count()) / ((float)(nextStamp - previousStamp).count());
        Eigen::Quaternion<float> currentOrientation = previousOrientation.slerp(ratio, nextOrientation);
        Eigen::Matrix<float, 3, 1> currentPosition = previousPosition + ratio * (nextPosition - previousPosition);
        Eigen::Matrix<float, 4, 4> currentPose = Eigen::Matrix<float, 4, 4>::Identity();
        currentPose.topLeftCorner<3, 3>() = currentOrientation.matrix();
        currentPose.topRightCorner<3, 1>() = currentPosition;

        deskewedCloud.features.col(i) = currentPose * skewedCloud.features.col(i);
    }

    return deskewedCloud;
}

std::vector<StampedState> applyTransformationToStates(const Eigen::Matrix<float, 4, 4>& transformation, const std::vector<StampedState>& states)
{
    std::vector<StampedState> transformedStates;
    for(unsigned int i = 0; i < states.size(); ++i)
    {
        transformedStates.push_back({states[i].timeStamp, transformation * states[i].pose, transformation.topLeftCorner<3, 3>() * states[i].velocity});
    }
    return transformedStates;
}
