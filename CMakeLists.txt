cmake_minimum_required(VERSION 3.8)
project(norlab_icp_mapper)

set(CMAKE_CXX_STANDARD 17)

set(EXTERNAL_INCLUDE_DIRS "")
set(EXTERNAL_LIBS "")

find_package(libpointmatcher CONFIG)
set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${libpointmatcher_LIBRARIES})

# norlab_icp_mapper target
include_directories(norlab_icp_mapper ${libpointmatcher_INCLUDE_DIRS})
add_library(norlab_icp_mapper norlab_icp_mapper/Mapper.cpp norlab_icp_mapper/Map.cpp norlab_icp_mapper/Trajectory.cpp norlab_icp_mapper/RAMCellManager.cpp norlab_icp_mapper/HardDriveCellManager.cpp)
target_link_libraries(norlab_icp_mapper ${libpointmatcher_LIBRARIES})

# install target
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(DEF_INSTALL_CMAKE_DIR lib/cmake/norlab_icp_mapper CACHE PATH "Installation directory for CMake files")

install(TARGETS norlab_icp_mapper DESTINATION ${INSTALL_LIB_DIR})

install(FILES norlab_icp_mapper/Mapper.h norlab_icp_mapper/Map.h  norlab_icp_mapper/Trajectory.h norlab_icp_mapper/CellManager.h
        DESTINATION ${INSTALL_INCLUDE_DIR}/norlab_icp_mapper
        )

configure_file(norlab_icp_mapperConfig.cmake.in
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/norlab_icp_mapperConfig.cmake" @ONLY
        )
install(FILES
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/norlab_icp_mapperConfig.cmake"
        DESTINATION ${DEF_INSTALL_CMAKE_DIR}
        COMPONENT dev
        )

#=============== trigger other makefile ======================

# Example programs
option(NORLAB_MAPPER_BUILD_EXAMPLES "Build norlab_icp_mapper examples" ON)
if (NORLAB_MAPPER_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# uninstall target
if (NOT TARGET uninstall)
    configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)
    add_custom_target(uninstall
            COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif ()

