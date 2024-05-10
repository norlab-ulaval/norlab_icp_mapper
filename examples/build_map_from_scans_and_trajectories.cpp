//
// Created by Matěj Boxan on 2023-11-10.
//

#include <norlab_icp_mapper/Mapper.h>
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

    std::istringstream header_stream(header);
    std::vector<std::string> column_names;
    std::string column_name;

    while(std::getline(header_stream, column_name, ','))
    {
        column_names.push_back(column_name);
    }

    // Find the indices of required columns
    int x_index = -1, y_index = -1, z_index = -1, qx_index = -1, qy_index = -1, qz_index = -1, qw_index = -1, sec_index = -1, nsec_index = -1;

    for(int i = 0; i < column_names.size(); ++i)
    {
        if(column_names[i] == "pose.pose.position.x")
        {
            x_index = i;
        }
        else if(column_names[i] == "pose.pose.position.y")
        {
            y_index = i;
        }
        else if(column_names[i] == "pose.pose.position.z")
        {
            z_index = i;
        }
        else if(column_names[i] == "pose.pose.orientation.x")
        {
            qx_index = i;
        }
        else if(column_names[i] == "pose.pose.orientation.y")
        {
            qy_index = i;
        }
        else if(column_names[i] == "pose.pose.orientation.z")
        {
            qz_index = i;
        }
        else if(column_names[i] == "pose.pose.orientation.w")
        {
            qw_index = i;
        }
        else if(column_names[i] == "header.stamp.sec")
        {
            sec_index = i;
        }
        else if(column_names[i] == "header.stamp.nanosec")
        {
            nsec_index = i;
        }
    }

    // Check if all required columns are found
    if(x_index == -1 || y_index == -1 || z_index == -1 || qx_index == -1 || qy_index == -1 || qz_index == -1 ||
       qw_index == -1 || sec_index == -1 || nsec_index == -1)
    {
        throw std::runtime_error("Error: Required columns not found in the header.");
    }

    // Define Eigen types for vectors and matrices
    using Vector3d = Eigen::Vector3d;
    using Quaterniond = Eigen::Quaterniond;
    using Affine3d = Eigen::Affine3d;

    // Containers to store data
    std::vector<std::pair<Affine3d, unsigned long>> stamped_transformations;

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
        for(int i = 0; i < column_names.size(); ++i)
        {
            std::getline(linestream, token, ',');
            std::stringstream ss;
            ss << token;
            if(i == x_index || i == y_index || i == z_index)
            {
                // Extract position (x, y, z)
                double value;
                ss >> value;
                if(i == x_index)
                {
                    position.x() = value;
                }
                else if(i == y_index)
                {
                    position.y() = value;
                }
                else if(i == z_index)
                {
                    position.z() = value;
                }
            }
            else if(i == qx_index || i == qy_index || i == qz_index || i == qw_index)
            {
                // Extract quaternion (x, y, z, w)
                double value;
                ss >> value;
                if(i == qx_index)
                {
                    quaternion.x() = value;
                }
                else if(i == qy_index)
                {
                    quaternion.y() = value;
                }
                else if(i == qz_index)
                {
                    quaternion.z() = value;
                }
                else if(i == qw_index)
                {
                    quaternion.w() = value;
                }
            }
            else if(i == sec_index || i == nsec_index)
            {
                // Extract timestamp (sec, nsec)
                unsigned long value;
                ss >> value;
                if(i == sec_index)
                {
                    timestamp += value * static_cast<unsigned long>(1e9);
                }
                else if(i == nsec_index)
                {
                    timestamp += value;
                }
            }
        }
        Affine3d transform;
        transform.translation() = position;
        transform.linear() = quaternion.toRotationMatrix();
        stamped_transformations.emplace_back(transform, timestamp);
    }
    return stamped_transformations;
}

std::vector<std::string> getScansPaths(const std::string& directory_path)
{

    // Vector to store file paths
    std::vector<std::string> vtk_files;

    // Iterate over the files in the directory
    for(const auto& entry: fs::directory_iterator(directory_path))
    {
        // Check if the file has a .vtk extension
        if(entry.path().extension() == ".vtk")
        {
            vtk_files.push_back(entry.path().string());
        }
    }

    std::sort(vtk_files.begin(), vtk_files.end());

    return vtk_files;
}

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cerr << "Please provide a dataPath as an argument." << std::endl;
        return -1;
    }

    fs::path dataPath = argv[1];
    auto stampedTransformations = getStampedTransformations(dataPath / "icp_odom.csv");
    auto vtk_files_paths = getScansPaths(dataPath / "scans/");

    assert(stampedTransformations.size() == vtk_files_paths.size());

    using namespace norlab_icp_mapper;

    auto mapper = std::make_unique<Mapper>(dataPath / "config.yaml",
                                           true, false,
                                           true, false);

    for(size_t i = 0; i < stampedTransformations.size(); ++i)
    {
        auto timestamp = std::chrono::time_point<std::chrono::steady_clock>(
                std::chrono::nanoseconds(stampedTransformations[i].second));
        PM::TransformationParameters transformationParameters(stampedTransformations[i].first.matrix().cast<float>());
        if(i == 0 || mapper->shouldUpdateMap(timestamp, transformationParameters, 0.0))
        {
            std::string inputPath = vtk_files_paths[i];
            DP inputCloud(DP::load(inputPath));

            mapper->processInput(inputCloud, transformationParameters, timestamp);
        }
    }

    fs::path output_path = dataPath / "map.vtk";
    mapper->getMap().save(output_path);
    std::cout << "Output saved to " << output_path << std::endl;

    return 0;
}


