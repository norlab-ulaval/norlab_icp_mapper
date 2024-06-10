#include "norlab_icp_mapper/Mapper.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <filesystem>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

namespace fs = std::filesystem;

std::vector<std::pair<Eigen::Affine3d, unsigned long>> getStampedTransformations(const std::string& filepath)
{
    // Open the CSV file
    std::ifstream file(filepath);

    if(!file.is_open())
    {
        throw std::runtime_error("Could not open file");
    }

    // Read the header line to get column indices
    std::string header;
    std::getline(file, header);

    std::istringstream headerStream(header);
    std::vector<std::string> columnNames;
    std::string columnName;

    while(std::getline(headerStream, columnName, ','))
    {
        columnNames.push_back(columnName);
    }

    // Find the indices of required columns
    int xIndex = -1, yIndex = -1, zIndex = -1, qxIndex = -1, qyIndex = -1, qzIndex = -1, qwIndex = -1, secIndex = -1, nsecIndex = -1;

    for(int i = 0; i < columnNames.size(); ++i)
    {
        if(columnNames[i] == "pose.pose.position.x")
        {
            xIndex = i;
        }
        else if(columnNames[i] == "pose.pose.position.y")
        {
            yIndex = i;
        }
        else if(columnNames[i] == "pose.pose.position.z")
        {
            zIndex = i;
        }
        else if(columnNames[i] == "pose.pose.orientation.x")
        {
            qxIndex = i;
        }
        else if(columnNames[i] == "pose.pose.orientation.y")
        {
            qyIndex = i;
        }
        else if(columnNames[i] == "pose.pose.orientation.z")
        {
            qzIndex = i;
        }
        else if(columnNames[i] == "pose.pose.orientation.w")
        {
            qwIndex = i;
        }
        else if(columnNames[i] == "header.stamp.sec")
        {
            secIndex = i;
        }
        else if(columnNames[i] == "header.stamp.nanosec")
        {
            nsecIndex = i;
        }
    }

    // Check if all required columns are found
    if(xIndex == -1 || yIndex == -1 || zIndex == -1 || qxIndex == -1 || qyIndex == -1 || qzIndex == -1 ||
       qwIndex == -1 || secIndex == -1 || nsecIndex == -1)
    {
        throw std::runtime_error("Error: Required columns not found in the header.");
    }

    // Define Eigen types for vectors and matrices
    using Vector3d = Eigen::Vector3d;
    using Quaterniond = Eigen::Quaterniond;
    using Affine3d = Eigen::Affine3d;

    // Containers to store data
    std::vector<std::pair<Affine3d, unsigned long>> stampedTransformations;

    // Read the file line by line
    std::string line;
    while(std::getline(file, line))
    {
        std::istringstream linestream(line);
        std::string token;

        Vector3d position;
        Quaterniond quaternion;
        unsigned long timestamp = 0;
        // Extract the required columns
        for(int i = 0; i < columnNames.size(); ++i)
        {
            std::getline(linestream, token, ',');
            std::stringstream ss;
            ss << token;
            if(i == xIndex || i == yIndex || i == zIndex)
            {
                // Extract position (x, y, z)
                double value;
                ss >> value;
                if(i == xIndex)
                {
                    position.x() = value;
                }
                else if(i == yIndex)
                {
                    position.y() = value;
                }
                else if(i == zIndex)
                {
                    position.z() = value;
                }
            }
            else if(i == qxIndex || i == qyIndex || i == qzIndex || i == qwIndex)
            {
                // Extract quaternion (x, y, z, w)
                double value;
                ss >> value;
                if(i == qxIndex)
                {
                    quaternion.x() = value;
                }
                else if(i == qyIndex)
                {
                    quaternion.y() = value;
                }
                else if(i == qzIndex)
                {
                    quaternion.z() = value;
                }
                else if(i == qwIndex)
                {
                    quaternion.w() = value;
                }
            }
            else if(i == secIndex || i == nsecIndex)
            {
                // Extract timestamp (sec, nsec)
                unsigned long value;
                ss >> value;
                if(i == secIndex)
                {
                    timestamp += value * static_cast<unsigned long>(1e9);
                }
                else if(i == nsecIndex)
                {
                    timestamp += value;
                }
            }
        }
        Affine3d transform;
        transform.translation() = position;
        transform.linear() = quaternion.toRotationMatrix();
        stampedTransformations.emplace_back(transform, timestamp);
    }
    return stampedTransformations;
}

std::vector<std::string> getScansPaths(const std::string& directoryPath)
{

    // Vector to store file paths
    std::vector<std::string> vtkFiles;

    // Iterate over the files in the directory
    for(const auto& entry: fs::directory_iterator(directoryPath))
    {
        // Check if the file has a .vtk extension
        if(entry.path().extension() == ".vtk")
        {
            vtkFiles.push_back(entry.path().string());
        }
    }

    std::sort(vtkFiles.begin(), vtkFiles.end());

    return vtkFiles;
}

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cerr << "Please provide a dataPath as an argument." << std::endl;
        return -1;
    }
    if(argc < 3)
    {
        std::cerr << "Please provide a yaml config as an argument." << std::endl;
        return -1;
    }

    fs::path dataPath = argv[1];
    fs::path configYaml = argv[2];
    auto stampedTransformations = getStampedTransformations(dataPath / "trajectory.csv");
    auto vtkFilesPaths = getScansPaths(dataPath / "scans/");

    assert(stampedTransformations.size() == vtkFilesPaths.size());

    using namespace norlab_icp_mapper;

    auto mapper = std::make_unique<Mapper>(configYaml,
                                           true, false,
                                           true, false);

    for(size_t i = 0; i < stampedTransformations.size(); ++i)
    {
        auto timestamp = std::chrono::time_point<std::chrono::steady_clock>(
                std::chrono::nanoseconds(stampedTransformations[i].second));
        PM::TransformationParameters transformationParameters(stampedTransformations[i].first.matrix().cast<float>());
        std::string inputPath = vtkFilesPaths[i];
        DP inputCloud(DP::load(inputPath));

        mapper->processInput(inputCloud, transformationParameters, timestamp);
    }

    fs::path outputPath = dataPath / "map.vtk";
    mapper->getMap().save(outputPath);
    std::cout << "Output saved to " << outputPath << std::endl;

    return 0;
}


