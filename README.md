# norlab_icp_mapper
A 2-D/3-D mapping library relying on the "Iterative Closest Point" algorithm.

## Dependencies
This branch of norlab_icp_mapper requires gtsam to function. In order to install this dependency, follow these steps:
```bash
cd ~/repos
git clone git@github.com:borglab/gtsam.git
cd gtsam
git checkout master
mkdir build
cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j 6
sudo make install
```
