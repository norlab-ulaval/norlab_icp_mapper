#include "mapper.h"
#include "norlab_icp_mapper/Mapper.h"

namespace python
{
    namespace module
    {
        void pybindMapperModule(py::module& p_module)
        {
            using Mapper = norlab_icp_mapper::Mapper;
            py::class_<Mapper>(p_module, "Mapper")
                    .def("processInput", &Mapper::processInput, py::arg("inputInSensorFrame"), py::arg("estimatedPose"), py::arg("timeStamp"))
                    .def("getMap", &Mapper::getMap)
                    .def("setMap", &Mapper::setMap, py::arg("newMap"))
                    .def("getNewLocalMap", &Mapper::getNewLocalMap, py::arg("mapOut"))
                    .def("getPose", &Mapper::getPose)
                    .def("getIsMapping", &Mapper::getIsMapping)
                    .def("setIsMapping", &Mapper::setIsMapping, py::arg("isMapping"))
                    .def("getTrajectory", &Mapper::getTrajectory)
                    .def("setDefaultMapperModule", &Mapper::setDefaultMapperModule)
                    .def("loadYamlConfig", &Mapper::loadYamlConfig, py::arg("yamlConfigFilePath"))
                    .def(py::init<const std::string&, const bool&, const bool&, const bool&, const bool&>(),
                         py::arg("configPath"), py::arg("is3D"), py::arg("isOnline"),
                         py::arg("isMapping"), py::arg("saveMapCellsOnHardDrive"), "Constructor");
        }
    }
} // python
