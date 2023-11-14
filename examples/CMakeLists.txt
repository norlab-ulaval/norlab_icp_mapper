include_directories(..)

add_executable(build_map_from_trajectory BuildMapFromTrajectory.cpp)
target_link_libraries(build_map_from_trajectory norlab_icp_mapper)

if (TARGET libnabo::nabo)
  target_link_libraries(build_map_from_trajectory libnabo::nabo)
else()
  include_directories(build_map_from_trajectory PUBLIC ${libnabo_INCLUDE_DIRS})
  target_link_libraries(build_map_from_trajectory ${libnabo_LIBRARIES})
endif()
