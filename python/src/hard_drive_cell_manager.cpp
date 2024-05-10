//
// Created by mbo on 19/05/22.
//

#include "hard_drive_cell_manager.h"
#include "norlab_icp_mapper/HardDriveCellManager.h"

namespace python
{
	namespace module
	{
		void pybindHardDriveCellManagerModule(py::module& p_module) {

			using HardDriveCellManager = norlab_icp_mapper::HardDriveCellManager;
			py::class_<HardDriveCellManager>(p_module, "HardDriveCellManager")
				.def("getAllCellIds", &HardDriveCellManager::getAllCellIds)
				.def("saveCell", &HardDriveCellManager::saveCell, py::arg("cellId"), py::arg("cell"))
				.def("retrieveCell", &HardDriveCellManager::retrieveCell, py::arg("cellId"))
				.def("clearAllCells", &HardDriveCellManager::clearAllCells)
				.def(py::init(), "Constructor");
		}
	}
}