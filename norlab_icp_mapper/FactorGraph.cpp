#include "FactorGraph.h"
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

FactorGraph::FactorGraph(const Eigen::Matrix<float, 4, 4>& initialLidarPose, const Eigen::Matrix<float, 3, 1>& initialLidarLinearVelocity,
                         const std::chrono::time_point<std::chrono::steady_clock>& initialTimeStamp, const std::chrono::time_point<std::chrono::steady_clock>& finalTimeStamp,
                         const std::vector<ImuMeasurement>& imuMeasurements, const Eigen::Matrix<float, 4, 4>& imuToLidar):
        imuMeasurements(imuMeasurements), imuToLidar(imuToLidar)
{
    Eigen::Matrix<double, 4, 4> initialImuPose = (initialLidarPose * imuToLidar).cast<double>();
    Eigen::Matrix<float, 3, 1> initialAngularVelocity = (initialLidarPose * imuToLidar).topLeftCorner<3, 3>() * imuMeasurements[0].angularVelocity;
    Eigen::Matrix<float, 3, 1> initialImuLeverArm = (initialLidarPose * imuToLidar).topRightCorner<3, 1>() - initialLidarPose.topRightCorner<3, 1>();
    Eigen::Matrix<double, 3, 1> initialImuLinearVelocity = (initialLidarLinearVelocity + initialAngularVelocity.cross(initialImuLeverArm)).cast<double>();

    nbPoses = imuMeasurements.size() + 1;
//    optimizerParams.setVerbosityLM("SUMMARY");
    optimizerParams.setlambdaUpperBound(1e8);
    imuBias = gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), gtsam::Vector3::Zero());

    // priors
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(1, gtsam::Pose3(initialImuPose), INITIAL_POSE_PRIOR_NOISE));
    initialEstimate.insert(1, gtsam::Pose3(initialImuPose));
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(nbPoses + 1, initialImuLinearVelocity, INITIAL_VELOCITY_PRIOR_NOISE));
    initialEstimate.insert(nbPoses + 1, initialImuLinearVelocity);
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(2 * nbPoses + 1, imuBias, BIAS_PRIOR_NOISE));
    initialEstimate.insert(2 * nbPoses + 1, imuBias);

    // IMU constraints
    poseStamps.push_back(initialTimeStamp);
    boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams = gtsam::PreintegrationParams::MakeSharedU();
    preintegrationParams->setGyroscopeCovariance(GYROSCOPE_COVARIANCE);
    preintegrationParams->setAccelerometerCovariance(ACCELEROMETER_COVARIANCE);
    preintegrationParams->setIntegrationCovariance(INTEGRATION_COVARIANCE);
    gtsam::PreintegratedImuMeasurements preintegrator(preintegrationParams, imuBias);
    gtsam::PreintegratedImuMeasurements predictor(preintegrationParams, imuBias);
    for(unsigned int i = 1; i < nbPoses; ++i)
    {
        if(i < nbPoses - 1)
        {
            poseStamps.push_back(imuMeasurements[i].timeStamp);
        }
        else
        {
            poseStamps.push_back(finalTimeStamp);
        }
        double dt = (poseStamps[poseStamps.size() - 1] - poseStamps[poseStamps.size() - 2]).count() / 1e9;

        preintegrator.integrateMeasurement(imuMeasurements[i - 1].linearAcceleration.cast<double>(), imuMeasurements[i - 1].angularVelocity.cast<double>(), dt);
        graph.add(gtsam::ImuFactor(i, nbPoses + i, i + 1, nbPoses + i + 1, 2 * nbPoses + 1, preintegrator));
        preintegrator.resetIntegration();

        predictor.integrateMeasurement(imuMeasurements[i - 1].linearAcceleration.cast<double>(), imuMeasurements[i - 1].angularVelocity.cast<double>(), dt);
        gtsam::NavState predictedState = predictor.predict(gtsam::NavState(gtsam::Pose3(initialImuPose), initialImuLinearVelocity), imuBias);
        initialEstimate.insert(i + 1, predictedState.pose());
        initialEstimate.insert(nbPoses + i + 1, predictedState.velocity());

        if(i == nbPoses - 1)
        {
            estimatedFinalLidarPose = predictedState.pose().matrix().cast<float>() * imuToLidar.inverse();
        }
    }
}

std::vector<StampedState> FactorGraph::getPredictedStates() const
{
    std::vector<StampedState> predictedStates;
    for(unsigned int i = 1; i < nbPoses + 1; ++i)
    {
        Eigen::Matrix<float, 4, 4> currentImuPose = initialEstimate.at<gtsam::Pose3>(i).matrix().cast<float>();
        Eigen::Matrix<float, 4, 4> currentLidarPose = currentImuPose * imuToLidar.inverse();
        ImuMeasurement currentImuMeasurement;
        if(i < nbPoses)
        {
            currentImuMeasurement = imuMeasurements[i - 1];
        }
        else
        {
            currentImuMeasurement = imuMeasurements[imuMeasurements.size() - 1];
        }
        Eigen::Matrix<float, 3, 1> currentAngularVelocity = currentImuPose.topLeftCorner<3, 3>() * currentImuMeasurement.angularVelocity;
        Eigen::Matrix<float, 3, 1> currentLidarLeverArm = currentLidarPose.topRightCorner<3, 1>() - currentImuPose.topRightCorner<3, 1>();
        Eigen::Matrix<float, 3, 1> currentLidarLinearVelocity = initialEstimate.at<gtsam::Vector3>(nbPoses + i).cast<float>() + currentAngularVelocity.cross(currentLidarLeverArm);
        predictedStates.push_back({poseStamps[i - 1], currentLidarPose, currentLidarLinearVelocity});
    }
    return predictedStates;
}

std::vector<StampedState> FactorGraph::optimize(const Eigen::Matrix<float, 4, 4>& registrationTransformation) const
{
    Eigen::Matrix<float, 4, 4> finalLidarPose = registrationTransformation * estimatedFinalLidarPose; // not 100% sure
    gtsam::Pose3 finalImuPose((finalLidarPose * imuToLidar).cast<double>());

    gtsam::NonlinearFactorGraph graphCopy(graph);
    graphCopy.add(gtsam::BetweenFactor<gtsam::Pose3>(1, nbPoses, finalImuPose, REGISTRATION_NOISE));

    gtsam::LevenbergMarquardtOptimizer optimizer(graphCopy, initialEstimate, optimizerParams);
    gtsam::Values result = optimizer.optimize();
    if(result.equals(initialEstimate, 1e-6))
    {
        throw std::runtime_error("The factor graph optimization did not converge!");
    }

    std::vector<StampedState> optimizedStates;
    for(unsigned int i = 1; i < nbPoses + 1; ++i)
    {
        Eigen::Matrix<float, 4, 4> currentImuPose = result.at<gtsam::Pose3>(i).matrix().cast<float>();
        Eigen::Matrix<float, 4, 4> currentLidarPose = currentImuPose * imuToLidar.inverse();
        ImuMeasurement currentImuMeasurement;
        if(i < nbPoses)
        {
            currentImuMeasurement = imuMeasurements[i - 1];
        }
        else
        {
            currentImuMeasurement = imuMeasurements[imuMeasurements.size() - 1];
        }
        Eigen::Matrix<float, 3, 1> currentAngularVelocity = currentImuPose.topLeftCorner<3, 3>() * currentImuMeasurement.angularVelocity;
        Eigen::Matrix<float, 3, 1> currentLidarLeverArm = currentLidarPose.topRightCorner<3, 1>() - currentImuPose.topRightCorner<3, 1>();
        Eigen::Matrix<float, 3, 1> currentLidarLinearVelocity = result.at<gtsam::Vector3>(nbPoses + i).cast<float>() + currentAngularVelocity.cross(currentLidarLeverArm);
        optimizedStates.push_back({poseStamps[i - 1], currentLidarPose, currentLidarLinearVelocity});
    }
    return optimizedStates;
}
