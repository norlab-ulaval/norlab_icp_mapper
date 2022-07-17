//
// Created by mbo on 19/05/22.
//

#include "trajectory.h"
#include "Trajectory.h"

namespace python
{
	namespace module
	{
		// TODO try memberClassInterface
		//  https://stackoverflow.com/questions/59088261/how-to-wrap-a-c-class-to-python-so-that-i-can-access-its-members-public-metho
		//  for private members
		void pybindTrajectoryModule(py::module& p_module) {
			py::class_<Trajectory>(p_module, "Trajectory")
				.def("addPoint", &Trajectory::addPoint, py::arg("point"))
				.def("save", &Trajectory::save, py::arg("filename"))
				.def("clearPoints", &Trajectory::clearPoints)
				.def(py::init<int>(), py::arg("dimension"), "Constructor");
		}
	}
}