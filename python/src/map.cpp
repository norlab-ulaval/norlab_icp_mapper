#include "map.h"
#include "norlab_icp_mapper/Map.h"

namespace python
{
    namespace module
    {
        void pybindMapModule(py::module& p_module)
        {
            using Map = norlab_icp_mapper::Map;
            py::class_<Map>(p_module, "Map")
                    .def("updatePose", &Map::updatePose, py::arg("pose"))
                    .def("getLocalPointCloud", &Map::getLocalPointCloud)
                    .def("updateLocalPointCloud", &Map::updateLocalPointCloud, py::arg("input"), py::arg("pose"), py::arg("postFilters"))
                    .def("getNewLocalPointCloud", &Map::getNewLocalPointCloud, py::arg("loalPointCloudOut"))
                    .def("getGlobalPointCloud", &Map::getGlobalPointCloud)
                    .def("setGlobalPointCloud", &Map::setGlobalPointCloud, py::arg("newLocalPointCloud"))
                    .def("isLocalPointCloudEmpty", &Map::isLocalPointCloudEmpty)
                    .def("addMapperModule", &Map::addMapperModule, py::arg("mapperModule"))
                    .def("computeDynamicPoints", &Map::computesDynamicPoints)
                    .def_property("sensorMaxRange", &Map::getSensorMaxRange, &Map::setSensorMaxRange)
                    .def(py::init<const bool&, const bool&, const bool&,
                            PM::ICPSequence&, std::mutex&>(),
                         py::arg("is3D"), py::arg("isOnline"), py::arg("saveCellsOnHardDrive"),
                         py::arg("icp"), py::arg("icpMapLock"), "Constuctor");
        }
    }
}