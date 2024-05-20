# Compiling and installing norlab_icp_mapper
This tutorial will guide you through the different steps to install norlab_icp_mapper.
The mapper relies on `libpointmatcher` for point cloud registration.
Check its [installation guide](https://libpointmatcher.readthedocs.io/en/latest/Compilation/) before continuing with this tutorial.
The mapper can possibly be deployed on other platforms, but the primary target is Ubuntu.

| Name           |     Version  <br> (Tested on our CI/CD server)     |       Version  <br> (Tested on our CI/CD server)        | Version  <br> (Tested on our CI/CD server) |
|:---------------|:--------------------------------------------------:|:-------------------------------------------------------:|:------------------------------------------:|
| Ubuntu         |            bionic 18.04.1 LTS (64 bit)             |                focal 20.04 LTS (64 bit)                 |          jammy 22.04 LTS (64 bit)          |
| Architecture   |                  x86 and arm64/v8                  |                     x86 and arm64/v8                    |              x86 and arm64/v8              |
   
!!! note

    we only support 64-bit systems because of some issues with Eigen. Other versions will most probably work but you'll have to try yourself to know for sure.

### Dependencies
norlab_icp_mapper relies principally on `libpointmatcher` and its dependencies. 

| Library name | cmake  | yaml-cpp | libpointmatcher |
|:-------------|:------:|:--------:|:---------------:|
| **Version**  | 3.10.2 |   0.5    |      1.4.2      |

### Installing norlab_icp_mapper

First, you need to clone the source repository into a local directory. As an example, we reuse the Libraries directory that was created to contain the libnabo sources.

```bash
cd ~/Libraries/
git clone git://github.com/norlab-ulaval/norlab_icp_mapper.git
cd norlab_icp_mapper
```

Create a build directory, set the build flag to release and set a flag to build examples

```bash
SRC_DIR=${PWD}
BUILD_DIR=${SRC_DIR}/build
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo -D BUILD_EXAMPLE=TRUE ${SRC_DIR}
```

Now, to compile /norlab_icp_mapper into the `/build` directory, run the following command:
```bash
make
```

#### Installation

Finally, to install norlab_icp_mapper on your system, run the following command:
```bash
sudo make install
```