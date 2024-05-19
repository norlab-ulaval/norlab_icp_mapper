#include "octree.h"
#include "norlab_icp_mapper/MapperModules/OctreeMapperModule.h"

namespace python
{
    namespace mappermodules
    {
        void pybindOctree(py::module& p_module)
        {
            py::class_<OctreeMapperModule, std::shared_ptr<OctreeMapperModule>, MapperModule>(p_module, "OctreeMapperModule")
                    .def_static("description", &OctreeMapperModule::description)
                    .def_static("availableParameters", &OctreeMapperModule::availableParameters)

                    .def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

                    .def("createMap", &OctreeMapperModule::createMap, py::arg("input"), py::arg("pose"))
                    .def("inPlaceCreateMap", &OctreeMapperModule::inPlaceCreateMap, py::arg("input"), py::arg("pose"))
                    .def("updateMap", &OctreeMapperModule::updateMap, py::arg("input"), py::arg("map"), py::arg("pose"))
                    .def("inPlaceUpdateMap", &OctreeMapperModule::inPlaceUpdateMap, py::arg("input"), py::arg("map"), py::arg("pose"));
        }
    }
}