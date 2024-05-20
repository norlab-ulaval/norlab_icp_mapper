# Using norlab_icp_mapper in Python

This guide assumes that you have `libpointmatcher` [Python bindings](https://libpointmatcher.readthedocs.io/en/latest/CompilationPython/) installed in your Python environment.
Also note that you need the mapper not only built, but also installed on your device, otherwise the Python binding will not find the library.
run the following commands to install Python bindings into your current python environment:
```bash
cd python
pip install .
```

Now test your installation with
```bash
python -c "from pynorlab_icp_mapper import *"
```

You can then run an example that builds a map from consecutive scans, similarly to the one described in [Example: building a map from lidar scans](RunningExample.md).
```bash
cd examples
pip install -r requirements.txt
python build_map_from_scans_and_trajectory.py
```
When prompted for a datapath, input `data`.
The default configuration file is located in `config.yaml`.
The final map will be saved in `data/map_python.vtk`.
![Final map light](images/example_map_light.png#only-light)
![Final map dark](images/example_map_dark.png#only-dark)