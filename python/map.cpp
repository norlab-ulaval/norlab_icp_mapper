//
// Created by mbo on 19/05/22.
//

#include "map.h"
#include "Map.h"

namespace python
{
	namespace module
	{
		void pybindMapModule(py::module& p_module) {
			using Map = norlab_icp_mapper::Map;
			py::class_<Map>(p_module, "Map")
			    .def("updatePose", &Map::updatePose, py::arg("pose"))
				.def("getLocalPointCloud", &Map::getLocalPointCloud)
				.def("updateLocalPointCloud", &Map::updateLocalPointCloud, py::arg("input"), py::arg("pose"), py::arg("postFilters"))
				.def("getNewLocalPointCloud", &Map::getNewLocalPointCloud, py::arg("loalPointCloudOut"))
				.def("getGlobalPointCloud", &Map::getGlobalPointCloud)
				.def("setGlobalPointCloud", &Map::setGlobalPointCloud, py::arg("newLocalPointCloud"))
				.def("isLocalPointCloudEmpty", &Map::isLocalPointCloudEmpty)
				.def(py::init<const float&, const float&, const float&, const float&, const float&,
			const float&, const float&, const float&, const float&, const bool&, const bool&,
			const bool&, const bool&, PM::ICPSequence&, std::mutex&>(),
						py::arg("minDistNewPoint"), py::arg("sensorMaxRange"), py::arg("priorDynamic"),
						py::arg("thresholdDynamic"), py::arg("beamHalfAngle"),
						py::arg("epsilonA"), py::arg("epsilonD"), py::arg("alpha"), py::arg("beta"), py::arg("is3D"),
						py::arg("isOnline"), py::arg("computeProbDynamic"), py::arg("saveCellsOnHardDrive"), py::arg("icp"),
						py::arg("icpMapLock"), "Constuctor");
		}
	}
}