#include "mapper_module.h"
#include "norlab_icp_mapper/MapperModules/MapperModule.h"

namespace python
{
    namespace mappermodules
    {
        void pybindMapperModule(py::module& p_module)
        {
            py::class_<MapperModule, std::shared_ptr<MapperModule>, PM::Parametrizable>(p_module, "MapperModule", "A MapperModule creates a map from a different output and then updates the map with scan-pose pairs.")
                    .def("createMap", &MapperModule::createMap, py::arg("input"), py::arg("pose"))
                    .def("inPlaceCreateMap", &MapperModule::inPlaceCreateMap, py::arg("input"), py::arg("pose"))
                    .def("updateMap", &MapperModule::updateMap, py::arg("input"), py::arg("map"), py::arg("pose"))
                    .def("inPlaceUpdateMap", &MapperModule::inPlaceUpdateMap, py::arg("input"), py::arg("map"), py::arg("pose"));
        }
    }
}