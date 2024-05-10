//
// Created by mbo on 19/05/22.
//

#include "cell_manager.h"
#include "norlab_icp_mapper/CellManager.h"

namespace python
{
	namespace module
	{
		void pybindCellManagerModule(py::module& p_module) {
			using CellManager = norlab_icp_mapper::CellManager;
			py::class_<CellManager>(p_module, "CellManager")
				.def("addPoint", &CellManager::getAllCellIds)
				.def("saveCell", &CellManager::saveCell, py::arg("cellId"), py::arg("cell"))
				.def("retrieveCell", &CellManager::retrieveCell, py::arg("cellId"))
				.def("clearAllCells", &CellManager::clearAllCells);
		}
	}
}
