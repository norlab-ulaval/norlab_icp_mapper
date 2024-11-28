#include "FactorGraph.h"
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include "utils.hpp"

const std::string OUTPUT_FOLDER = "/home/sp/Desktop/debug_2/";

FactorGraph::FactorGraph(const Eigen::Matrix<float, 4, 4>& initialLidarPose, const Eigen::Matrix<float, 3, 1>& initialLidarLinearVelocity,
                         const std::chrono::time_point<std::chrono::steady_clock>& initialTimeStamp, const std::chrono::time_point<std::chrono::steady_clock>& finalTimeStamp,
                         const std::vector<ImuMeasurement>& imuMeasurements, const Eigen::Matrix<float, 4, 4>& imuToLidar, const bool& reconstructContinuousTrajectory,
                         const float& linearVelocityNoise):
        imuMeasurements(imuMeasurements), imuToLidar(imuToLidar), reconstructContinuousTrajectory(reconstructContinuousTrajectory)
{
    initialImuPose = (initialLidarPose * imuToLidar).cast<double>();
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
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(nbPoses + 1, initialImuLinearVelocity, gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector::Ones(3) * linearVelocityNoise)));
    initialEstimate.insert(nbPoses + 1, initialImuLinearVelocity);
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(2 * nbPoses + 1, imuBias, BIAS_PRIOR_NOISE));
    initialEstimate.insert(2 * nbPoses + 1, imuBias);

    // IMU constraints
    poseStamps.push_back(initialTimeStamp);
    boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams = gtsam::PreintegrationParams::MakeSharedU(); // the map frame needs to be aligned with gravity
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
            estimatedFinalImuPose = predictedState.pose().matrix();
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

std::vector<StampedState> FactorGraph::optimize(const Eigen::Matrix<float, 4, 4>& registrationTransformation, const int& iterationCounter, const bool& saveGraph) const
{
    if(reconstructContinuousTrajectory)
    {
        Eigen::Matrix<double, 4, 4> finalImuPose = registrationTransformation.cast<double>() * estimatedFinalImuPose;
        gtsam::Pose3 registrationConstraint(initialImuPose.inverse() * finalImuPose);

        gtsam::NonlinearFactorGraph graphCopy(graph);
        graphCopy.add(gtsam::BetweenFactor<gtsam::Pose3>(1, nbPoses, registrationConstraint, REGISTRATION_NOISE));

        if(saveGraph)
        {
            save(initialEstimate, registrationTransformation, OUTPUT_FOLDER + "initial_estimate_" + std::to_string(iterationCounter) + ".csv");
        }

        gtsam::LevenbergMarquardtOptimizer optimizer(graphCopy, initialEstimate, optimizerParams);
        gtsam::Values result = optimizer.optimize();

        if(saveGraph)
        {
            save(result, registrationTransformation, OUTPUT_FOLDER + "optimization_result_" + std::to_string(iterationCounter) + ".csv");
        }

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
    else
    {
        std::vector<StampedState> optimizedStates = applyTransformationToStates(registrationTransformation, getPredictedStates());
        Eigen::Matrix<float, 3, 1> initialLidarPosition = (initialImuPose.cast<float>() * imuToLidar.inverse()).topRightCorner<3, 1>();
        Eigen::Matrix<float, 3, 1> finalLidarPosition = (registrationTransformation * estimatedFinalImuPose.cast<float>() * imuToLidar.inverse()).topRightCorner<3, 1>();
        double dt = (optimizedStates[optimizedStates.size() - 1].timeStamp - optimizedStates[0].timeStamp).count() / 1e9;
        Eigen::Matrix<float, 3, 1> finalLidarVelocity = (finalLidarPosition - initialLidarPosition) / dt;
        optimizedStates[optimizedStates.size() - 1].velocity = finalLidarVelocity;
        return optimizedStates;
    }
}

#include <fstream>

void FactorGraph::save(const gtsam::Values& values, const Eigen::Matrix<float, 4, 4>& registrationTransformation, const std::string& fileName) const
{
    Eigen::Matrix<double, 4, 4> finalImuPose = registrationTransformation.cast<double>() * estimatedFinalImuPose;

    std::ofstream outputFile(fileName);
    outputFile << "t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33" << std::endl;
    outputFile << "# IMU poses" << std::endl;
    for(unsigned int i = 1; i < nbPoses + 1; ++i)
    {
        Eigen::Matrix<float, 4, 4> currentImuPose = values.at<gtsam::Pose3>(i).matrix().cast<float>();
        outputFile << currentImuPose(0, 0) << "," << currentImuPose(0, 1) << "," << currentImuPose(0, 2) << "," << currentImuPose(0, 3) << "," <<
                   currentImuPose(1, 0) << "," << currentImuPose(1, 1) << "," << currentImuPose(1, 2) << "," << currentImuPose(1, 3) << "," <<
                   currentImuPose(2, 0) << "," << currentImuPose(2, 1) << "," << currentImuPose(2, 2) << "," << currentImuPose(2, 3) << "," <<
                   currentImuPose(3, 0) << "," << currentImuPose(3, 1) << "," << currentImuPose(3, 2) << "," << currentImuPose(3, 3) << std::endl;
    }
    outputFile << "# registration constraint" << std::endl;
    outputFile << finalImuPose(0, 0) << "," << finalImuPose(0, 1) << "," << finalImuPose(0, 2) << "," << finalImuPose(0, 3) << "," <<
               finalImuPose(1, 0) << "," << finalImuPose(1, 1) << "," << finalImuPose(1, 2) << "," << finalImuPose(1, 3) << "," <<
               finalImuPose(2, 0) << "," << finalImuPose(2, 1) << "," << finalImuPose(2, 2) << "," << finalImuPose(2, 3) << "," <<
               finalImuPose(3, 0) << "," << finalImuPose(3, 1) << "," << finalImuPose(3, 2) << "," << finalImuPose(3, 3) << std::endl;
    outputFile.close();
}
