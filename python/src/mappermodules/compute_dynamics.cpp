#include "compute_dynamics.h"
#include "norlab_icp_mapper/MapperModules/ComputeDynamicsMapperModule.h"

namespace python
{
    namespace mappermodules
    {
        void pybindComputeDynamics(py::module& p_module)
        {
            py::class_<ComputeDynamicsMapperModule, std::shared_ptr<ComputeDynamicsMapperModule>, MapperModule>(p_module, "ComputeDynamicsMapperModule")
                    .def_static("description", &ComputeDynamicsMapperModule::description)
                    .def_static("availableParameters", &ComputeDynamicsMapperModule::availableParameters)

                    .def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

                    .def("createMap", &ComputeDynamicsMapperModule::createMap, py::arg("input"), py::arg("pose"))
                    .def("inPlaceCreateMap", &ComputeDynamicsMapperModule::inPlaceCreateMap, py::arg("input"), py::arg("pose"))
                    .def("updateMap", &ComputeDynamicsMapperModule::updateMap, py::arg("input"), py::arg("map"), py::arg("pose"))
                    .def("inPlaceUpdateMap", &ComputeDynamicsMapperModule::inPlaceUpdateMap, py::arg("input"), py::arg("map"), py::arg("pose"))
                    .def("convertToSphericalCoordinates", &ComputeDynamicsMapperModule::convertToSphericalCoordinates, py::arg("points"), py::arg("radii"), py::arg("angles"));
        }
    }
}