cmake_minimum_required(VERSION 3.15)
project(src/pynorlab_icp_mapper)
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
    find_package(Eigen3 REQUIRED NO_MODULE)
    include_directories(${EIGEN3_INCLUDE_DIR})
    find_package(libpointmatcher REQUIRED)
    find_package(norlab_icp_mapper REQUIRED)

    find_package(Python COMPONENTS Interpreter Development REQUIRED)
    if (Python_FOUND)
        message(STATUS "Python found: ${Python_VERSION}")
    else()
        message(FATAL_ERROR "Python not found")
    endif()

    # Use Python to locate pybind11 installed via pip
    execute_process(COMMAND ${Python_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
                    OUTPUT_VARIABLE pybind11_DIR
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    find_package(pybind11 CONFIG REQUIRED)

    if (pybind11_FOUND) #${SKBUILD_PROJECT_NAME}
        pybind11_add_module(_core ${PYBIND11_SOURCES})

        target_link_libraries(_core
                PUBLIC
                norlab_icp_mapper
                ${norlab_icp_mapper_LIBRARIES}
                ${libpointmatcher_LIBRARIES})

        add_dependencies(_core norlab_icp_mapper)

        install(TARGETS _core LIBRARY DESTINATION ${SKBUILD_PROJECT_NAME})
    else ()
        message(FATAL_ERROR "pybind11 is required!")
    endif ()
endif ()
