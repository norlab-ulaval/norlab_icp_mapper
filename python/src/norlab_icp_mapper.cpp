#include "cell_manager.h"
#include "ram_cell_manager.h"
#include "hard_drive_cell_manager.h"
#include "map.h"
#include "mapper.h"
#include "trajectory.h"
#include "mappermodules/mapper_module.h"
#include "mappermodules/dynamic_points.h"
#include "mappermodules/octree.h"
#include "mappermodules/point_distance.h"

PYBIND11_MODULE(_core, module)
{
    py::module::import("pypointmatcher");

    module.doc() = "Python bindings of norlab_icp_mapper";

    python::module::pybindCellManagerModule(module);
    python::module::pybindRAMCellManagerModule(module);
    python::module::pybindHardDriveCellManagerModule(module);
    python::module::pybindMapModule(module);
    python::module::pybindMapperModule(module);
    python::module::pybindTrajectoryModule(module);

    py::module mapperModulesModule = module.def_submodule("mappermodules");
    python::mappermodules::pybindMapperModule(mapperModulesModule);
    python::mappermodules::pybindDynamicPoints(mapperModulesModule);
    python::mappermodules::pybindOctree(mapperModulesModule);
    python::mappermodules::pybindPointDistance(mapperModulesModule);
}