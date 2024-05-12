#include "trajectory.h"
#include "norlab_icp_mapper/Trajectory.h"

namespace python
{
    namespace module
    {
        void pybindTrajectoryModule(py::module& p_module)
        {
            py::class_<Trajectory>(p_module, "Trajectory")
                    .def("addPose", &Trajectory::addPose, py::arg("pose"), py::arg("timeStamp"))
                    .def("save", &Trajectory::save, py::arg("filename"))
                    .def("clear", &Trajectory::clear)
                    .def(py::init<int>(), py::arg("dimension"), "Constructor");
        }
    }
}