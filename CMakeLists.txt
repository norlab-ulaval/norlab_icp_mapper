# 3.15 is the minimum version for scikit-build-core
cmake_minimum_required(VERSION 3.15)

# We need helpers to generate a version config file
include(CMakePackageConfigHelpers)

project(norlab_icp_mapper)

# # Extract version from package.xml
file(READ "package.xml" PACKAGE_XML_CONTENT)
string(REGEX MATCH "<version>([^<]+)</version>" VERSION_MATCH ${PACKAGE_XML_CONTENT})
# Extract the matched version from the captured group
if(VERSION_MATCH)
    # CMake variable ${CMAKE_MATCH_1} contains the matched version
    set(PROJECT_VERSION ${CMAKE_MATCH_1})
    message(STATUS "Found package version: ${PROJECT_VERSION}")
else()
    message(SEND_ERROR "Package version not found in package.xml")
endif()

set(CMAKE_CXX_STANDARD 17)

set(EXTERNAL_INCLUDE_DIRS "")
set(EXTERNAL_LIBS "")

set(CMAKE_CXX_FLAGS                " ${CMAKE_CXX_FLAGS_INIT} -std=c++17 -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG          "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DDEBUG")

message(STATUS "Looking for libpointmatcher on system...")
find_package(libpointmatcher CONFIG 1.4.3 REQUIRED)
set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${libpointmatcher_LIBRARIES})

message(STATUS "Looking for yaml-cpp on system...")
find_package(yaml-cpp CONFIG REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_LIBRARIES "yaml-cpp::yaml-cpp")
endif()
if(YAML_CPP_LIBRARIES STREQUAL "")
    set(YAML_CPP_LIBRARIES "yaml-cpp")
endif ()

# norlab_icp_mapper target
add_library(norlab_icp_mapper
        norlab_icp_mapper/Mapper.cpp
        norlab_icp_mapper/Map.cpp
        norlab_icp_mapper/Trajectory.cpp
        norlab_icp_mapper/RAMCellManager.cpp
        norlab_icp_mapper/HardDriveCellManager.cpp

#       MapperModules
        norlab_icp_mapper/MapperModules/PointDistanceMapperModule.cpp
        norlab_icp_mapper/MapperModules/DynamicPointsMapperModule.cpp
        norlab_icp_mapper/MapperModules/OctreeMapperModule.cpp
        norlab_icp_mapper/MapperModules/VoxelDownsamplingMapperModule.cpp
        norlab_icp_mapper/MapperModules/MapperModule.cpp
        )
target_include_directories(norlab_icp_mapper PUBLIC
        ${libpointmatcher_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/norlab_icp_mapper>
        )
target_link_libraries(norlab_icp_mapper
        ${libpointmatcher_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

set_target_properties(norlab_icp_mapper PROPERTIES
        VERSION "${PROJECT_VERSION}"
        SOVERSION 1
)

# install target
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(DEF_INSTALL_CMAKE_DIR lib/cmake/norlab_icp_mapper CACHE PATH "Installation directory for CMake files")

install(TARGETS norlab_icp_mapper DESTINATION ${INSTALL_LIB_DIR})

install(FILES norlab_icp_mapper/Mapper.h
        norlab_icp_mapper/Map.h
        norlab_icp_mapper/Trajectory.h
        norlab_icp_mapper/CellManager.h
        norlab_icp_mapper/RAMCellManager.h
        norlab_icp_mapper/HardDriveCellManager.h
        DESTINATION ${INSTALL_INCLUDE_DIR}/norlab_icp_mapper
        )
# Install Mapper Modules
install(FILES
        norlab_icp_mapper/MapperModules/MapperModule.h
        norlab_icp_mapper/MapperModules/DynamicPointsMapperModule.h
        norlab_icp_mapper/MapperModules/OctreeMapperModule.h
        norlab_icp_mapper/MapperModules/PointDistanceMapperModule.h
        norlab_icp_mapper/MapperModules/VoxelDownsamplingMapperModule.h
        DESTINATION ${INSTALL_INCLUDE_DIR}/norlab_icp_mapper/MapperModules/)

configure_file(norlab_icp_mapperConfig.cmake.in
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/norlab_icp_mapperConfig.cmake" @ONLY
        )

# The same versioning file can be used for both cases
write_basic_package_version_file(norlab_icp_mapperConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)

install(FILES
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/norlab_icp_mapperConfig.cmake"
     "${PROJECT_BINARY_DIR}/norlab_icp_mapperConfigVersion.cmake"
        DESTINATION ${DEF_INSTALL_CMAKE_DIR}
        COMPONENT dev
        )

# uninstall target
if (NOT TARGET uninstall)
    configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)
    add_custom_target(uninstall
            COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif ()


set(BUILD_EXAMPLE false CACHE BOOL "Set to true to build an example program")
if(BUILD_EXAMPLE)
    message(STATUS "Building example")
    add_executable(build_map_from_scans_and_trajectories examples/build_map_from_scans_and_trajectory.cpp)
    target_link_libraries(build_map_from_scans_and_trajectories norlab_icp_mapper)
endif()
