import datetime
import os
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path
from pypointmatcher import pointmatcher as pm
from pynorlab_icp_mapper import Mapper
from tqdm import tqdm

PM = pm.PointMatcher
DP = PM.DataPoints

def get_stamped_transformations(filepath):
    # Open the CSV file
    file = open(filepath, 'r')

    # Read the header line to get column indices
    header = file.readline().strip().split(',')

    column_names = header

    # Find the indices of required columns
    x_index = -1
    y_index = -1
    z_index = -1
    qx_index = -1
    qy_index = -1
    qz_index = -1
    qw_index = -1
    sec_index = -1
    nsec_index = -1

    for i in range(len(column_names)):
        if column_names[i] == "pose.pose.position.x":
            x_index = i
        elif column_names[i] == "pose.pose.position.y":
            y_index = i
        elif column_names[i] == "pose.pose.position.z":
            z_index = i
        elif column_names[i] == "pose.pose.orientation.x":
            qx_index = i
        elif column_names[i] == "pose.pose.orientation.y":
            qy_index = i
        elif column_names[i] == "pose.pose.orientation.z":
            qz_index = i
        elif column_names[i] == "pose.pose.orientation.w":
            qw_index = i
        elif column_names[i] == "header.stamp.sec":
            sec_index = i
        elif column_names[i] == "header.stamp.nanosec":
            nsec_index = i

    # Check if all required columns are found
    if (x_index == -1 or y_index == -1 or z_index == -1 or qx_index == -1 or
            qy_index == -1 or qz_index == -1 or qw_index == -1 or sec_index == -1 or
            nsec_index == -1):
        raise Exception("Error: Required columns not found in the header.")

    # Containers to store data
    stamped_transformations = []

    # Read the file line by line
    csv_reader = csv.reader(file)
    file_content = list(csv_reader)

    for row in file_content:
        position = np.array([0, 0, 0], dtype=float)
        quaternion = np.array([0, 0, 0, 0], dtype=float)
        timestamp = datetime.timedelta()

        for i in range(len(column_names)):
            if i == x_index or i == y_index or i == z_index:
                # Extract position (x, y, z)
                value = float(row[i])

                if i == x_index:
                    position[0] = value
                elif i == y_index:
                    position[1] = value
                elif i == z_index:
                    position[2] = value

            elif i == qx_index or i == qy_index or i == qz_index or i == qw_index:
                # Extract quaternion (x, y, z, w)
                value = float(row[i])

                if i == qx_index:
                    quaternion[0] = value
                elif i == qy_index:
                    quaternion[1] = value
                elif i == qz_index:
                    quaternion[2] = value
                elif i == qw_index:
                    quaternion[3] = value

            elif i == sec_index or i == nsec_index:
                # Extract timestamp (sec, nsec)
                value = int(row[i])

                if i == sec_index:
                    timestamp += datetime.timedelta(seconds=value)
                elif i == nsec_index:
                    timestamp += datetime.timedelta(microseconds=int(value/1000))

        rotation = R.from_quat(quaternion).as_matrix()
        stamped_transformations.append((position, rotation, timestamp))

    return stamped_transformations


def get_scans_paths(directory_path):
    # Vector to store file paths
    vtk_files = []

    # Iterate over the files in the directory
    for entry in Path(directory_path).iterdir():
        # Check if the file has a .vtk extension
        if entry.is_file() and entry.suffix == ".vtk":
            vtk_files.append(str(entry))

    vtk_files.sort()

    return vtk_files


if __name__ == "__main__":
    dataPath = os.path.relpath(input("Please provide a data path: "))
    stamped_transformations = get_stamped_transformations(os.path.join(dataPath, "icp_odom.csv"))
    vtk_files_paths = get_scans_paths(os.path.join(dataPath, "scans"))

    assert len(stamped_transformations) == len(vtk_files_paths)

    # Mapper configuration
    inputFiltersConfigFilePath = ""
    icpConfigFilePath = ""
    mapPostFiltersConfigFilePath = ""
    mapUpdateCondition = "distance"
    mapUpdateOverlap = 0.9
    mapUpdateDelay = 1.0
    mapUpdateDistance = 0.0
    minDistNewPoint = 0.0
    sensorMaxRange = 100.0
    priorDynamic = 0.6
    thresholdDynamic = 0.9
    beamHalfAngle = 0.01
    epsilonA = 0.01
    epsilonD = 0.01
    alpha = 0.8
    beta = 0.99
    is3D = True
    isOnline = False
    computeProbDynamic = False
    isMapping = True
    saveMapCellsOnHardDrive = False

    mapper = Mapper(inputFiltersConfigFilePath, icpConfigFilePath, mapPostFiltersConfigFilePath,
                    mapUpdateCondition, mapUpdateOverlap, mapUpdateDelay, mapUpdateDistance,
                    minDistNewPoint, sensorMaxRange, priorDynamic, thresholdDynamic, beamHalfAngle,
                    epsilonA, epsilonD, alpha, beta, is3D, isOnline, computeProbDynamic,
                    isMapping, saveMapCellsOnHardDrive)

    for i in tqdm(range(len(stamped_transformations))):
        timestamp = stamped_transformations[i][2]
        T = np.identity(4, dtype=np.float32)
        T[0:3, 0:3] = stamped_transformations[i][1]
        T[0:3, 3] = stamped_transformations[i][0]
        inputPath = vtk_files_paths[i]
        inputCloud = DP.load(inputPath)

        mapper.processInput(inputCloud, T, timestamp)

    output_path = os.path.join(dataPath, "output.vtk")
    print(f"Success! Map saved to {output_path}")
    mapper.getMap().save(output_path)

