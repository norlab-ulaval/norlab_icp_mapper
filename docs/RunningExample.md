# Running norlab_icp_mapper example

The library comes with an example that builds a map from individual scans and a trajectory.
Use this toy example to get familiar with the mapper's configuration before jumping to integrating the mapper with ROS.

The example is located in `examples/build_map_from_scans_and_trajectory.cpp` and is build with the `-D BUILD_EXAMPLE=TRUE` flag.
After building the library, you can execute it on the provided data:
```shell
cd build
build_map_from_scans_and_trajectory ../examples/data ../examples/config.yaml
```
The script will process the data in the given directory and generate a file `examples/data/map.vtk`.
Use [ParaView](https://www.paraview.org/) or other point cloud viewer to inspect it.

The `config.yaml` file contains configuration for the mapper, as well as for the registration algorithm handled by `libpointmatcher`.
Check the [MapperConfiguration](MapperConfiguration.md) tutorial for more details.

The default final map looks something like this:
![Final map light](images/example_map_light.png#only-light)
![Final map dark](images/example_map_dark.png#only-dark)
### Input data format
In case you want to run the mapper offline on your data, you can have a look in the `examples/data` folder for the required format.

##### Scans
The scans (point clouds readings) are located in the `examples/data/scans` folder and can be in any format [supported by libpointmatcher](https://libpointmatcher.readthedocs.io/en/latest/ImportExport/).
The file name must start with `cloud_`, followed by a timestamp representing the `seconds` and `nanoseconds` of the data recording.
For example, from the two first scans: `cloud_1690309709_285305600.vtk` and `cloud_1690309709_385259008.vtk` you can notice that they were taken with approximately 100 ms delay, indicating a sensor operating at 10 Hz.

##### Trajectories
Trajectories are described in `examples/data/trajectory.csv`.
The file format corresponds to the definition of the [ROS PoseStamped message](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html).
Importantly, the `header.stamp.sec` and `header.stamp.nanosec` columns must match the scan file names.
No interpolation in either scans or poses is happening, the number of rows (without the header) in the .csv file must exactly match the number of files in `examples/data/scans`. 

##### How to get these files from a rosbag?
The mapper currently doesn't support the processing of rosbag files.
For this feature, take a look at the [norlab_icp_mapper_ros](https://github.com/norlab-ulaval/norlab_icp_mapper_ros) bridge.
However, it is possible to generate files in the required format from an existing rosbag file.
In fact, that is exactly how the files in this guide were created!
On a system with either ROS or ROS 2 installed, download a script that exports a ROS message type into .csv and execute it:

=== "ROS"
    ```shell
    wget https://raw.githubusercontent.com/norlab-ulaval/norlab_scripts/main/extract_topic_to_csv.sh
    extract_topic_to_csv.sh input.bag /odometry trajectory.csv
    ```
=== "ROS 2"
    ```shell
    wget https://raw.githubusercontent.com/norlab-ulaval/norlab_scripts/main/extract_topic_to_csv_ros2.py
    python3 extract_topic_to_csv_ros2.py bagfolder trajectory.csv /odometry
    ```
This extract the topic `/odometry` from a rosbag (or bag folder in ROS 2) into `trajectory.csv`.

To get the scans from your bagfile, you'll need to have `libpointmatcher` installed with its Python bindings.
Check [this](https://libpointmatcher.readthedocs.io/en/latest/CompilationPython/) installation guide how to do that.
Execute the following code, which fetches the exporting script and then runs it, waiting for incoming message on topic `/lidar_data`.
```shell
wget https://raw.githubusercontent.com/norlab-ulaval/norlab_scripts/main/save_vtk.py
save_vtk.py /lidar_data scans
```
In a second terminal, play your rosbag file:

=== "ROS"
    ```shell
    rosbag play input.bag
    ```
=== "ROS 2"
    ```shell
    ros2 bag play bagfolder
    ```
Your scans will be exported into the `scans` folder.