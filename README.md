# norlab_icp_mapper
A 2-D/3-D mapping library relying on the "Iterative Closest Point" algorithm.

### Build and Installation guide
```bash
mkdir build && cd build
cmake ..
sudo make install
```

### Python bindings
Assuming that you have `libpointmatcher` [Python bindings](https://libpointmatcher.readthedocs.io/en/latest/CompilationPython/) installed, run the following commands to install Python bindings into your current python environment:
```bash
cd python
pip install .
```
Note that you need the mapper installed on your device, otherwise the Python binding will not find the library.

### Documentation
The documentation for both users and developers is hosted on [readthedocs.org](https://norlab-icp-mapper.readthedocs.io/en/latest/).
Alternatively, it can be found in the `doc/` folder, however without the nice formatting possible thanks to [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/).