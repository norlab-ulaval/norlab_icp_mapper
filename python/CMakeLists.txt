if("${PYTHON_INSTALL_TARGET}" STREQUAL "")
	message(STATUS "PYTHON_INSTALL_TARGET variable is not set, setting a default value...")
	execute_process(COMMAND "python3" "-c" "import site; print(site.getsitepackages()[0])"
	                OUTPUT_VARIABLE PYTHON_INSTALL_TARGET OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

message(STATUS "The Python module will be install at this location : ${PYTHON_INSTALL_TARGET}")

set(PYBIND11_SOURCES
		#main module
		norlab_icp_mapper.cpp
		cell_manager.cpp
		hard_drive_cell_manager.cpp
		ram_cell_manager.cpp
		map.cpp
		mapper.cpp
		trajectory.cpp
		)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(USE_SYSTEM_PYBIND11)
	find_package(pybind11 2.5.0 REQUIRED)
	message(STATUS "pybind11 v${pybind11_VERSION}")
else()
	add_subdirectory(pybind11)
	set(pybind11_FOUND TRUE)
endif()
find_package(libpointmatcher)

if(pybind11_FOUND)
	pybind11_add_module(pynorlabicpmapper ${PYBIND11_SOURCES})

	target_link_libraries(pynorlabicpmapper
	                      PUBLIC
						norlab_icp_mapper
			${libpointmatcher_LIBRARIES})

	add_dependencies(pynorlabicpmapper norlab_icp_mapper)

	install(TARGETS pynorlabicpmapper LIBRARY DESTINATION ${PYTHON_INSTALL_TARGET})
else()
	message(FATAL_ERROR "pybind11 is required!")
endif()

