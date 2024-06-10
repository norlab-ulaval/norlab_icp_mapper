#include "dynamic_points.h"
#include "norlab_icp_mapper/MapperModules/DynamicPointsMapperModule.h"

namespace python
{
    namespace mappermodules
    {
        void pybindDynamicPoints(py::module& p_module)
        {
            py::class_<DynamicPointsMapperModule, std::shared_ptr<DynamicPointsMapperModule>, MapperModule>(p_module, "DynamicPointsMapperModule")
                    .def_static("description", &DynamicPointsMapperModule::description)
                    .def_static("availableParameters", &DynamicPointsMapperModule::availableParameters)

                    .def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

                    .def("createMap", &DynamicPointsMapperModule::createMap, py::arg("input"), py::arg("pose"))
                    .def("inPlaceCreateMap", &DynamicPointsMapperModule::inPlaceCreateMap, py::arg("input"), py::arg("pose"))
                    .def("updateMap", &DynamicPointsMapperModule::updateMap, py::arg("input"), py::arg("map"), py::arg("pose"))
                    .def("inPlaceUpdateMap", &DynamicPointsMapperModule::inPlaceUpdateMap, py::arg("input"), py::arg("map"), py::arg("pose"))
                    .def("convertToSphericalCoordinates", &DynamicPointsMapperModule::convertToSphericalCoordinates, py::arg("points"), py::arg("radii"), py::arg("angles"));
        }
    }
}