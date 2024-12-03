#include <iostream>
#include <fstream>
#include <pointmatcher/PointMatcher.h>
#include <dirent.h>
#include "StampedState.h"
#include "Map.h"
#include "FactorGraph.h"
#include "utils.hpp"

const std::string ICP_CONFIG_FILE_PATH = "/home/sp/repos/norlab_icp_mapper/params/icp_config_cube_journal.yaml";
const PM::TransformationParameters IMU_TO_LIDAR = (PM::TransformationParameters(4, 4)
        << 0.0, -1.0, 0.0, 0.087, 1.0, 0.0, 0.0, -0.02, 0.0, 0.0, 1.0, -0.04, 0.0, 0.0, 0.0, 1.0).finished();
const bool RECONSTRUCT_CONTINUOUS_TRAJECTORY = false;

const std::string SCAN_FILE_NAME = "/home/sp/data/iros2024/debug/continuous_traj_run_1_decomposed/scan_1689175621955010048.csv";
const std::string MAP_FILE_NAME = "/home/sp/data/iros2024/debug/continuous_traj_run_1_decomposed/map_1689175621955010048.csv";
const std::string INITIAL_POSE_FILE_NAME = "/home/sp/data/iros2024/debug/continuous_traj_run_1_decomposed/initial_pose_1689175621955010048.csv";
const std::string INITIAL_VELOCITY_FILE_NAME = "/home/sp/data/iros2024/debug/continuous_traj_run_1_decomposed/initial_velocity_1689175621955010048.csv";
const std::string IMU_MEASUREMENTS_FOLDER = "/home/sp/data/iros2024/debug/continuous_traj_run_1_decomposed/";
const std::string FINAL_TIMESTAMP_FILE_NAME = "/home/sp/data/iros2024/debug/continuous_traj_run_1_decomposed/final_timestamp_1689175621955010048.csv";

typedef PointMatcher<float> PM;

PM::TransformationParameters loadInitialPose(const std::string& fileName)
{
    PM::TransformationParameters initialPose = PM::TransformationParameters::Zero(4, 4);
    std::string line;
    size_t lineCounter = 0;
    std::ifstream ifstream(fileName);
    while(std::getline(ifstream, line))
    {
        ++lineCounter;
        if(lineCounter == 2)
        {
            std::string token;
            size_t currentCursorPosition = 0;
            size_t nextCommaPosition = 0;
            size_t tokenCounter = 0;
            while(nextCommaPosition != std::string::npos)
            {
                nextCommaPosition = line.find(',', currentCursorPosition);
                token = line.substr(currentCursorPosition, nextCommaPosition - currentCursorPosition);
                currentCursorPosition = nextCommaPosition + 1;

                initialPose(tokenCounter / 4, tokenCounter % 4) = std::stof(token);
                ++tokenCounter;
            }
            break;
        }
    }
    ifstream.close();
    return initialPose;
}

Eigen::Matrix<float, 3, 1> loadInitialVelocity(const std::string& fileName)
{
    Eigen::Matrix<float, 3, 1> initialVelocity = Eigen::Matrix<float, 3, 1>::Zero();
    std::string line;
    size_t lineCounter = 0;
    std::ifstream ifstream(fileName);
    while(std::getline(ifstream, line))
    {
        ++lineCounter;
        if(lineCounter == 2)
        {
            std::string token;
            size_t currentCursorPosition = 0;
            size_t nextCommaPosition = 0;
            size_t tokenCounter = 0;
            while(nextCommaPosition != std::string::npos)
            {
                nextCommaPosition = line.find(',', currentCursorPosition);
                token = line.substr(currentCursorPosition, nextCommaPosition - currentCursorPosition);
                currentCursorPosition = nextCommaPosition + 1;

                initialVelocity(tokenCounter, 0) = std::stof(token);
                ++tokenCounter;
            }
            break;
        }
    }
    ifstream.close();
    return initialVelocity;
}

std::chrono::time_point<std::chrono::steady_clock> extractInitialTimestampFromScanFileName(const std::string& scanFileName)
{
    size_t lastUnderscorePosition = scanFileName.rfind('_');
    size_t fileExtensionPosition = scanFileName.rfind('.');
    std::string timestampString = scanFileName.substr(lastUnderscorePosition + 1, fileExtensionPosition - lastUnderscorePosition - 1);
    return std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(std::stol(timestampString)));
}

std::chrono::time_point<std::chrono::steady_clock> loadFinalTimestamp(const std::string& fileName)
{
    std::chrono::time_point<std::chrono::steady_clock> finalTimeStamp;
    std::string line;
    size_t lineCounter = 0;
    std::ifstream ifstream(fileName);
    while(std::getline(ifstream, line))
    {
        ++lineCounter;
        if(lineCounter == 1)
        {
            finalTimeStamp = std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(std::stol(line)));
            break;
        }
    }
    ifstream.close();
    return finalTimeStamp;
}

ImuMeasurement loadImuMeasurement(const std::string& fileName, const std::chrono::time_point<std::chrono::steady_clock>& timestamp)
{
    Eigen::Matrix<float, 3, 1> linearAcceleration;
    Eigen::Matrix<float, 3, 1> angularVelocity;
    std::string line;
    size_t lineCounter = 0;
    std::ifstream ifstream(fileName);
    while(std::getline(ifstream, line))
    {
        ++lineCounter;
        if(lineCounter == 2)
        {
            std::string token;
            size_t currentCursorPosition = 0;
            size_t nextCommaPosition = 0;
            size_t tokenCounter = 0;
            while(nextCommaPosition != std::string::npos)
            {
                nextCommaPosition = line.find(',', currentCursorPosition);
                token = line.substr(currentCursorPosition, nextCommaPosition - currentCursorPosition);
                currentCursorPosition = nextCommaPosition + 1;

                if(tokenCounter < 3)
                {
                    linearAcceleration(tokenCounter, 0) = std::stof(token);
                }
                else
                {
                    angularVelocity(tokenCounter-3, 0) = std::stof(token);
                }
                ++tokenCounter;
            }
            break;
        }
    }
    ifstream.close();
    return ImuMeasurement{timestamp, angularVelocity, linearAcceleration};
}

std::vector<ImuMeasurement> loadImuMeasurements(const std::string& folderName, const std::chrono::time_point<std::chrono::steady_clock>& initialTimestamp,
                                                const std::chrono::time_point<std::chrono::steady_clock>& finalTimestamp)
{
    std::vector<std::string> imuMeasurementFileNames;
    DIR* directory;
    struct dirent* entry;
    if((directory = opendir(folderName.c_str())) != NULL)
    {
        while((entry = readdir(directory)) != NULL)
        {
            std::string fileName = std::string(entry->d_name);
            if(fileName.find("imu_measurement_") != std::string::npos)
            {
                imuMeasurementFileNames.push_back(fileName);
            }
        }
        closedir(directory);
    }
    else
    {
        std::cerr << "Unable to load IMU measurements" << std::endl;
        exit(1);
    }

    std::sort(imuMeasurementFileNames.begin(), imuMeasurementFileNames.end());
    std::vector<ImuMeasurement> imuMeasurements;
    bool insideTimestampRange = false;
    for(size_t i = 0; i < imuMeasurementFileNames.size(); ++i)
    {
        if(!insideTimestampRange)
        {
            if(i < imuMeasurementFileNames.size() - 1)
            {
                size_t lastUnderscorePosition = imuMeasurementFileNames[i + 1].rfind('_');
                size_t fileExtensionPosition = imuMeasurementFileNames[i + 1].rfind('.');
                std::string timestampString = imuMeasurementFileNames[i + 1].substr(lastUnderscorePosition + 1, fileExtensionPosition - lastUnderscorePosition - 1);
                std::chrono::time_point<std::chrono::steady_clock> nextImuMeasurementTimeStamp(std::chrono::nanoseconds(std::stol(timestampString)));
                if(nextImuMeasurementTimeStamp > initialTimestamp)
                {
                    insideTimestampRange = true;
                    lastUnderscorePosition = imuMeasurementFileNames[i].rfind('_');
                    fileExtensionPosition = imuMeasurementFileNames[i].rfind('.');
                    timestampString = imuMeasurementFileNames[i].substr(lastUnderscorePosition + 1, fileExtensionPosition - lastUnderscorePosition - 1);
                    std::chrono::time_point<std::chrono::steady_clock> currentImuMeasurementTimeStamp(std::chrono::nanoseconds(std::stol(timestampString)));
                    imuMeasurements.push_back(loadImuMeasurement(folderName + imuMeasurementFileNames[i], currentImuMeasurementTimeStamp));
                }
            }
        }
        else
        {
            size_t lastUnderscorePosition = imuMeasurementFileNames[i].rfind('_');
            size_t fileExtensionPosition = imuMeasurementFileNames[i].rfind('.');
            std::string timestampString = imuMeasurementFileNames[i].substr(lastUnderscorePosition + 1, fileExtensionPosition - lastUnderscorePosition - 1);
            std::chrono::time_point<std::chrono::steady_clock> currentImuMeasurementTimeStamp(std::chrono::nanoseconds(std::stol(timestampString)));
            if(currentImuMeasurementTimeStamp >= finalTimestamp)
            {
                break;
            }
            imuMeasurements.push_back(loadImuMeasurement(folderName + imuMeasurementFileNames[i], currentImuMeasurementTimeStamp));
        }
    }
    return imuMeasurements;
}

int main(int argc, char** argv)
{
    PM::ICPSequence icp;

    if(!ICP_CONFIG_FILE_PATH.empty())
    {
        std::ifstream ifs(ICP_CONFIG_FILE_PATH.c_str());
        icp.loadFromYaml(ifs);
        ifs.close();
    }
    else
    {
        icp.setDefault();
    }

    PM::DataPoints reading = PM::DataPoints::load(SCAN_FILE_NAME);
    PM::DataPoints reference(PM::DataPoints::load(MAP_FILE_NAME));
    PM::TransformationParameters poseAtStartOfScan = loadInitialPose(INITIAL_POSE_FILE_NAME);
    Eigen::Matrix<float, 3, 1> velocityAtStartOfScan = loadInitialVelocity(INITIAL_VELOCITY_FILE_NAME);
    std::chrono::time_point<std::chrono::steady_clock> timeStampAtStartOfScan = extractInitialTimestampFromScanFileName(SCAN_FILE_NAME);
    std::chrono::time_point<std::chrono::steady_clock> timeStampAtEndOfScan = loadFinalTimestamp(FINAL_TIMESTAMP_FILE_NAME);
    std::vector<ImuMeasurement> imuMeasurements = loadImuMeasurements(IMU_MEASUREMENTS_FOLDER, timeStampAtStartOfScan, timeStampAtEndOfScan);

    std::vector<StampedState> optimizedStates;
    if(reference.getNbPoints() == 0)
    {
        FactorGraph factorGraph(poseAtStartOfScan, velocityAtStartOfScan, timeStampAtStartOfScan, timeStampAtEndOfScan, imuMeasurements, IMU_TO_LIDAR,
                                RECONSTRUCT_CONTINUOUS_TRAJECTORY, 1.0);
        optimizedStates = factorGraph.getPredictedStates();
    }
    else
    {
        icp.referenceDataPointsFilters.init();
        icp.referenceDataPointsFilters.apply(reference);
        const PM::Vector meanRef = reference.features.rowwise().sum() / reference.features.cols();
        PM::TransformationParameters T_refIn_refMean = PM::Matrix::Identity(4, 4);
        T_refIn_refMean.block(0, 3, 3, 1) = meanRef.head(3);
        reference.features.topRows(3).colwise() -= meanRef.head(3);
        std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create(icp.matcher->className, icp.matcher->parameters);
        matcher->init(reference);

        PM::TransformationParameters poseAtStartOfScan_refMean = T_refIn_refMean.inverse() * poseAtStartOfScan;
        FactorGraph factorGraph(poseAtStartOfScan_refMean, velocityAtStartOfScan, timeStampAtStartOfScan, timeStampAtEndOfScan, imuMeasurements, IMU_TO_LIDAR,
                                RECONSTRUCT_CONTINUOUS_TRAJECTORY, 1.0);
        std::vector<StampedState> optimizedStates_refMean = factorGraph.getPredictedStates();

        icp.readingDataPointsFilters.init();
        icp.readingDataPointsFilters.apply(reading);

        icp.readingStepDataPointsFilters.init();
        PM::TransformationParameters T_iter = PM::Matrix::Identity(4, 4);
        bool iterate(true);
        icp.transformationCheckers.init(T_iter, iterate);
        size_t iterationCount(0);
//        std::cout << "==== ICP residual ====" << std::endl;
//        PM::DataPoints stepReading(deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean));
//        PM::Matches matches(matcher->findClosests(stepReading));
//        PM::OutlierWeights outlierWeights(icp.outlierFilters.compute(stepReading, reference, matches));
//        std::cout << icp.errorMinimizer->getResidualError(stepReading, reference, outlierWeights, matches) << std::endl;
        while(iterate)
        {
            PM::DataPoints stepReading(deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean));
            icp.readingStepDataPointsFilters.apply(stepReading);
            PM::Matches matches(matcher->findClosests(stepReading));
            PM::OutlierWeights outlierWeights(icp.outlierFilters.compute(stepReading, reference, matches));
//            stepReading = deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean);
//            icp.readingStepDataPointsFilters.apply(stepReading);
//            matches = matcher->findClosests(stepReading);
//            outlierWeights = icp.outlierFilters.compute(stepReading, reference, matches);
            icp.inspector->dumpIteration(iterationCount, T_iter, reference, stepReading, matches, outlierWeights, icp.transformationCheckers);
            T_iter = icp.errorMinimizer->compute(stepReading, reference, outlierWeights, matches) * T_iter;
            optimizedStates_refMean = factorGraph.optimize(T_iter, iterationCount, true);
            std::cout << "Correction direction:" << std::endl << T_iter.topRightCorner<3, 1>() << std::endl << std::endl;
//            std::cout << icp.errorMinimizer->getResidualError(deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean), reference, outlierWeights, matches) << std::endl;
            try
            {
                icp.transformationCheckers.check(T_iter, iterate);
            }
            catch(...)
            {
                iterate = false;
            }
            ++iterationCount;
        }
//        std::cout << "ICP converged in " << iterationCount << " iterations" << std::endl;
//        std::cout << "=============================" << std::endl;

        optimizedStates = applyTransformationToStates(T_refIn_refMean, optimizedStates_refMean);
    }

    Eigen::Vector3f firstPosition = optimizedStates[0].pose.topRightCorner<3, 1>();
    Eigen::Vector3f lastPosition = optimizedStates[optimizedStates.size() - 1].pose.topRightCorner<3, 1>();
    std::cout << "The estimated displacement during the scan is " << (lastPosition - firstPosition).norm() << " meters" << std::endl;

    return 0;
}