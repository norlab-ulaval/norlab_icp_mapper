//
// Created by mbo on 19/05/22.
//

#include "mapper.h"
#include "Mapper.h"

namespace python
{
	namespace module
	{
		void pybindMapperModule(py::module& p_module) {
			using Mapper = norlab_icp_mapper::Mapper;
			py::class_<Mapper>(p_module, "Mapper")
				.def("loadYamlConfig", &Mapper::loadYamlConfig, py::arg("inputFiltersConfigFilePath"),
					py::arg("icpConfigFilePath"), py::arg("mapPostFiltersConfigFilePath"))
				.def("processInput", &Mapper::processInput, py::arg("inputInSensorFrame"), py::arg("estimatedPose"), py::arg("timeStamp"))
				.def("getMap", &Mapper::getMap)
				.def("setMap", &Mapper::setMap, py::arg("newMap"))
				.def("getNewLocalMap", &Mapper::getNewLocalMap, py::arg("mapOut"))
				.def("getPose", &Mapper::getPose)
				.def("getIsMapping", &Mapper::getIsMapping)
				.def("setIsMapping", &Mapper::setIsMapping, py::arg("newIsMapping"))
				.def("getTrajectory", &Mapper::getTrajectory)
				.def(py::init<const std::string&, const std::string&, const std::string&,
			const std::string&, const float&, const float&, const float&,
			const float&, const float&, const float&, const float&, const float&,
			const float&, const float&, const float&, const float&, const bool&, const bool&,
			const bool&, const bool&, const bool&>(), "Constructor", py::arg("inputFiltersConfigFilePath"), py::arg("icpConfigFilePath"),
					py::arg("mapPostFiltersConfigFilePath"), py::arg("mapUpdateCondition"), py::arg("mapUpdateOverlap"),
					py::arg("mapUpdateDelay"), py::arg("mapUpdateDistance"), py::arg("minDistNewPoint"), py::arg("sensorMaxRange"),
					py::arg("priorDynamic"), py::arg("thresholdDynamic"), py::arg("beamHalfAngle"),
					py::arg("epsilonA"), py::arg("epsilonD"), py::arg("alpha"), py::arg("beta"), py::arg("is3D"), py::arg("isOnline"),
					py::arg("computeProbDynamic"), py::arg("isMapping"), py::arg("saveMapCellsOnHardDrive"));
		}
	}
} // python