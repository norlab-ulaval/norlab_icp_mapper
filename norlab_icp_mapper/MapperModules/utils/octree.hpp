#include "octree.h"

#include <omp.h>

template <typename T, std::size_t dim>
Octree_<T, dim>::Octree_(std::size_t maxDataByNode, T maxSizeByNode, bool runInParallel) : maxDataByNode(maxDataByNode), maxSizeByNode(maxSizeByNode), runInParallel(runInParallel) {
	root = new Node<T, dim>(this);
}


template <typename T, std::size_t dim>
Octree_<T, dim>::~Octree_() {
	delete root;
}


template <typename T, std::size_t dim>
bool Octree_<T, dim>::build(const DP& pts) {
	deletedDataFromLastModification = std::vector<Id>();
	return root->build(pts, maxDataByNode, maxSizeByNode, runInParallel);
}

template <typename T, std::size_t dim>
bool Octree_<T, dim>::insert(const DP& newPts) {
	deletedDataFromLastModification = std::vector<Id>();
	return root->insert(newPts, maxDataByNode, maxSizeByNode, runInParallel);
}

template <typename T, std::size_t dim>
void Octree_<T, dim>::registerDeletedData(const std::vector<Id>& deletedData) {
	#pragma omp critical
	{
		deletedDataFromLastModification.insert(deletedDataFromLastModification.end(), deletedData.begin(), deletedData.end());
	}
}


//------------------------------------------------------------------------------
template <typename T, std::size_t dim>
template <typename Callback>
bool Octree_<T, dim>::visit(Callback& cb) {
	return root->visit(cb);
}
