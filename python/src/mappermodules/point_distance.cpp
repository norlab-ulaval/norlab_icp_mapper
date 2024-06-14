#include "point_distance.h"
#include "norlab_icp_mapper/MapperModules/PointDistanceMapperModule.h"

namespace python
{
    namespace mappermodules
    {
        void pybindPointDistance(py::module& p_module)
        {
            py::class_<PointDistanceMapperModule, std::shared_ptr<PointDistanceMapperModule>, MapperModule>(p_module, "PointDistanceMapperModule")
                    .def_static("description", &PointDistanceMapperModule::description)
                    .def_static("availableParameters", &PointDistanceMapperModule::availableParameters)

                    .def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

                    .def("createMap", &PointDistanceMapperModule::createMap, py::arg("input"), py::arg("pose"))
                    .def("inPlaceCreateMap", &PointDistanceMapperModule::inPlaceCreateMap, py::arg("input"), py::arg("pose"))
                    .def("updateMap", &PointDistanceMapperModule::updateMap, py::arg("input"), py::arg("map"), py::arg("pose"))
                    .def("inPlaceUpdateMap", &PointDistanceMapperModule::inPlaceUpdateMap, py::arg("input"), py::arg("map"), py::arg("pose"));
        }
    }
}