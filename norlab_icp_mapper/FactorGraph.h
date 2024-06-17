#ifndef NORLAB_ICP_MAPPER_FACTORGRAPH_H
#define NORLAB_ICP_MAPPER_FACTORGRAPH_H

#include <Eigen/Core>
#include <vector>
#include "ImuMeasurement.h"
#include "StampedState.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>

class FactorGraph
{
public:
    FactorGraph(const Eigen::Matrix<float, 4, 4>& initialLidarPose, const Eigen::Matrix<float, 3, 1>& initialLidarLinearVelocity,
                const std::chrono::time_point<std::chrono::steady_clock>& initialTimeStamp, const std::chrono::time_point<std::chrono::steady_clock>& finalTimeStamp,
                const std::vector<ImuMeasurement>& imuMeasurements, const Eigen::Matrix<float, 4, 4>& imuToLidar);
    std::vector<StampedState> getPredictedStates() const;
    std::vector<StampedState> optimize(const Eigen::Matrix<float, 4, 4>& registrationTransformation, const int& iterationCounter, const bool& saveGraph) const;
    void save(const gtsam::Values& values, const Eigen::Matrix<float, 4, 4>& registrationTransformation, const std::string& fileName) const;

private:
    const gtsam::SharedDiagonal INITIAL_POSE_PRIOR_NOISE = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector::Ones(6) * 1e-12);
    const gtsam::SharedDiagonal INITIAL_VELOCITY_PRIOR_NOISE = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector::Ones(3));
    const gtsam::SharedDiagonal BIAS_PRIOR_NOISE = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector::Ones(6) * 1e-12);
    const gtsam::Matrix GYROSCOPE_COVARIANCE = gtsam::Matrix::Identity(3, 3) * 2.7416e-5;
    const gtsam::Matrix ACCELEROMETER_COVARIANCE = gtsam::Matrix::Identity(3, 3) * 3.4645e-5;
    const gtsam::Matrix INTEGRATION_COVARIANCE = gtsam::Matrix::Identity(3, 3) * 1e-14;
    const gtsam::SharedDiagonal REGISTRATION_NOISE = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector::Ones(6) * 1e-12);

    Eigen::Matrix<double, 4, 4> initialImuPose;
    Eigen::Matrix<double, 4, 4> estimatedFinalImuPose;
    std::vector<ImuMeasurement> imuMeasurements;
    Eigen::Matrix<float, 4, 4> imuToLidar;
    unsigned int nbPoses;
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> poseStamps;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    gtsam::LevenbergMarquardtParams optimizerParams;
    gtsam::imuBias::ConstantBias imuBias;
};

#endif
