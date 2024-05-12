#include "cell_manager.h"
#include "ram_cell_manager.h"
#include "hard_drive_cell_manager.h"
#include "map.h"
#include "mapper.h"
#include "trajectory.h"

PYBIND11_MODULE(_core, module)
{
    module.doc() = "Python bindings of norlab_icp_mapper";

    python::module::pybindCellManagerModule(module);
    python::module::pybindRAMCellManagerModule(module);
    python::module::pybindHardDriveCellManagerModule(module);
    python::module::pybindMapModule(module);
    python::module::pybindMapperModule(module);
    python::module::pybindTrajectoryModule(module);
}