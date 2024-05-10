# norlab_icp_mapper
A 2-D/3-D mapping library relying on the "Iterative Closest Point" algorithm.

### Python bindings
Assuming that you have `libpointmatcher` [Python bindings](https://libpointmatcher.readthedocs.io/en/latest/CompilationPython/) installed, run the following commands to install Python bindings into your current python environment:
```bash
mkdir build && cd build
cmake ..
make install
pip install ..
```

Now test your installation with
```bash
python -c "from pynorlab_icp_mapper import *"
```

You can then test an example that builds a map from consecutive scans.
```bash
cd examples
pip install -r requirements.txt
python build_map_from_scans_and_trajectory.py
```
When prompted for a datapath, input `data`. The final map will be saved in `data/output.vtk`.

You can also try with your own data, the lidar point clouds must be located in a `scans` folder and their names must follow the "cloud_<sec>_<nanosec>" convention. The data format can be anything supported by [libpointmatcher](https://libpointmatcher.readthedocs.io/en/latest/ImportExport/). Additionally, you also need to provide a trajectory. See the `icp_odom.csv` file for an example.

