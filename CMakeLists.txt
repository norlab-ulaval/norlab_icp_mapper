cmake_minimum_required(VERSION 3.8)
project(norlab_icp_mapper)

set(CMAKE_CXX_STANDARD 17)

set(EXTERNAL_INCLUDE_DIRS "")
set(EXTERNAL_LIBS "")

message(STATUS "Looking for libpointmatcher on system...")
find_package(libpointmatcher CONFIG REQUIRED)
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

#        MapperModules
        norlab_icp_mapper/MapperModules/PointDistanceMapperModule.cpp
        norlab_icp_mapper/MapperModules/ComputeDynamicsMapperModule.cpp
        norlab_icp_mapper/MapperModules/OctreeMapperModule.cpp
        norlab_icp_mapper/MapperModules/MapperModule.cpp
        )
target_include_directories(norlab_icp_mapper PUBLIC
        ${libpointmatcher_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        )
target_link_libraries(norlab_icp_mapper
        ${libpointmatcher_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

# install target
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(DEF_INSTALL_CMAKE_DIR lib/cmake/norlab_icp_mapper CACHE PATH "Installation directory for CMake files")

install(TARGETS norlab_icp_mapper DESTINATION ${INSTALL_LIB_DIR})

install(FILES
        norlab_icp_mapper/Mapper.h
        norlab_icp_mapper/Map.h
        norlab_icp_mapper/Trajectory.h
        norlab_icp_mapper/CellManager.h
        DESTINATION ${INSTALL_INCLUDE_DIR}/norlab_icp_mapper
        )
install(FILES
        norlab_icp_mapper/MapperModules/MapperModule.h
        DESTINATION ${INSTALL_INCLUDE_DIR}/norlab_icp_mapper/MapperModules/)

configure_file(norlab_icp_mapperConfig.cmake.in
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/norlab_icp_mapperConfig.cmake" @ONLY
        )
install(FILES
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/norlab_icp_mapperConfig.cmake"
        DESTINATION ${DEF_INSTALL_CMAKE_DIR}
        COMPONENT dev
        )

#=============== trigger other makefile ======================

# uninstall target
if (NOT TARGET uninstall)
    configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)
    add_custom_target(uninstall
            COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif ()

