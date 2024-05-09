//
// Created by mbo on 19/05/22.
//

#include "trajectory.h"

namespace python
{
	namespace module
	{
		// TODO try memberClassInterface
		//  https://stackoverflow.com/questions/59088261/how-to-wrap-a-c-class-to-python-so-that-i-can-access-its-members-public-metho
		//  for private members
		void pybindTrajectoryModule(py::module& p_module) {
			py::class_<Trajectory>(p_module, "Trajectory")
				.def("addPose", &Trajectory::addPose, py::arg("pose"), py::arg("timeStamp"))
				.def("save", &Trajectory::save, py::arg("filename"))
				.def("clear", &Trajectory::clear)
				.def(py::init<int>(), py::arg("dimension"), "Constructor");
		}
	}
}