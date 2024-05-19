#include "mapper_module.h"
#include "compute_dynamics.h"
#include "octree.h"
#include "point_distance.h"

namespace python
{
    namespace modules
    {
        void pybindMapperModulesModule(py::module& p_module)
        {
            py::module mapperModulesModule = p_module.def_submodule("mappermodules");

            mappermodules::pybindComputeDynamics(mapperModulesModule);
            mappermodules::pybindOctree(mapperModulesModule);
            mappermodules::pybindPointDistance(mapperModulesModule);
        }
    }
}