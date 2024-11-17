cmake_minimum_required(VERSION 3.15...3.26)
project(src/pynorlab_icp_mapper
    LANGUAGES CXX
    VERSION "${SKBUILD_PROJECT_VERSION}")

set(PYBIND11_SOURCES
        #main module
        src/norlab_icp_mapper.cpp
        src/cell_manager.cpp
        src/hard_drive_cell_manager.cpp
        src/ram_cell_manager.cpp
        src/map.cpp
        src/mapper.cpp
        src/trajectory.cpp

        #mapper modules module
        src/mappermodules/mapper_module.cpp
        src/mappermodules/dynamic_points.cpp
        src/mappermodules/octree.cpp
        src/mappermodules/point_distance.cpp
)

include_directories(.)

if (DEFINED SKBUILD_PROJECT_NAME)

    set(PYBIND11_NEWPYTHON ON)

    find_package(pybind11 CONFIG REQUIRED)
    find_package(Eigen3 REQUIRED NO_MODULE)
    include_directories(${EIGEN3_INCLUDE_DIR})
    find_package(libpointmatcher REQUIRED)
    find_package(norlab_icp_mapper REQUIRED)
    if (pybind11_FOUND) #${SKBUILD_PROJECT_NAME}
        pybind11_add_module(_core ${PYBIND11_SOURCES})

        target_link_libraries(_core
                PUBLIC
                ${norlab_icp_mapper_LIBRARIES}
                ${libpointmatcher_LIBRARIES})

        add_dependencies(_core norlab_icp_mapper)

        install(TARGETS _core LIBRARY DESTINATION ${SKBUILD_PROJECT_NAME})
    else ()
        message(FATAL_ERROR "pybind11 is required!")
    endif ()
endif ()
