// kate: replace-tabs off; indent-width 5; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "node.h"

#include <omp.h>

template <typename T, std::size_t dim>
Node<T, dim>::Node(const Octree_<T, dim>* tree) : tree(tree) {
    for (size_t i = 0; i < nbCells; ++i) cells[i] = nullptr;
}

template <typename T, std::size_t dim>
Node<T, dim>::Node(const Node<T, dim>& o) : bb{o.bb.center, o.bb.radius} {
    if (o.isLeaf())  // Leaf case
    {
        // nullify childs
        for (size_t i = 0; i < nbCells; ++i) cells[i] = nullptr;
        // Copy data
        data.insert(data.end(), o.data.begin(), o.data.end());
    } else  // Node case
    {
        // Create each child recursively
        for (size_t i = 0; i < nbCells; ++i) {
            cells[i] = new Node<T, dim>(*(o.cells[i]));
        }
    }
}
template <typename T, std::size_t dim>
Node<T, dim>::Node(Node<T, dim>&& o) : bb{o.bb.center, o.bb.radius} {
    // only allow move of root node
    assert(o.getIsRoot());

    if (o.isLeaf())  // Leaf case
    {
        // Copy data
        data.insert(data.end(), std::make_move_iterator(o.data.begin()), std::make_move_iterator(o.data.end()));
    }
    // copy child ptr
    for (size_t i = 0; i < nbCells; ++i) {
        cells[i] = o.cells[i];
        // Nullify ptrs
        o.cells[i] = nullptr;
    }
}

template <typename T, std::size_t dim>
Node<T, dim>::~Node() {
    if (!isLeaf()) {
        for (int i = 0; i < nbCells; ++i) {
            delete cells[i];
        }
    }
}

template <typename T, std::size_t dim>
Node<T, dim>& Node<T, dim>::operator=(const Node<T, dim>& o) {

    if (o.isLeaf())  // Leaf case
    {
        // nullify childs
        for (size_t i = 0; i < nbCells; ++i) cells[i] = nullptr;
        // Copy data
        data.insert(data.end(), o.data.begin(), o.data.end());
    } else  // Node case
    {
        // Create each child recursively
        for (size_t i = 0; i < nbCells; ++i) {
            cells[i] = new Node<T, dim>(*(o.cells[i]));
        }
    }
    return *this;
}

template <typename T, std::size_t dim>
Node<T, dim>& Node<T, dim>::operator=(Node<T, dim>&& o) {
    // only allow move of root node
    assert(o.getIsRoot());

    bb.center = o.bb.center;
    bb.radius = o.bb.radius;


    if (o.isLeaf())  // Leaf case
    {
        // Copy data
        data.insert(data.end(), std::make_move_iterator(o.data.begin()), std::make_move_iterator(o.data.end()));
    }

    // copy childs ptrs
    for (size_t i = 0; i < nbCells; ++i) {
        cells[i] = o.cells[i];
        // Nullify ptrs
        o.cells[i] = nullptr;
    }

    return *this;
}

template <typename T, std::size_t dim>
bool Node<T, dim>::isLeaf() const {
    return (cells[0] == nullptr);
}
template <typename T, std::size_t dim>
bool Node<T, dim>::isEmpty() const {
    return (data.size() == 0);
}

template <typename T, std::size_t dim>
bool Node<T, dim>::isInsideBB(const DP& pts) const {
    typedef typename PM::Vector Vector;

    Vector minValues = pts.features.rowwise().minCoeff();
    Vector maxValues = pts.features.rowwise().maxCoeff();

    Point min = minValues.head(dim);
    Point max = maxValues.head(dim);

    for (size_t i = 0; i < dim; ++i) {
        if (min(i) <= bb.center(i) - bb.radius) return false;
        if (max(i) >= bb.center(i) + bb.radius) return false;
    }

    return true;
}

template <typename T, std::size_t dim>
std::size_t Node<T, dim>::idx(const Point& pt) const {
    size_t id = 0;

    for (size_t i = 0; i < dim; ++i) id |= ((pt(i) > bb.center(i)) << i);

    return id;
}

template <typename T, std::size_t dim>
std::size_t Node<T, dim>::idx(const DP& pts, const Id id) const {
    return idx(pts.features.col(id));
}
template <typename T, std::size_t dim>
T Node<T, dim>::getRadius() const {
    return bb.radius;
}
template <typename T, std::size_t dim>
typename Node<T, dim>::Point Node<T, dim>::getCenter() const {
    return bb.center;
}
template <typename T, std::size_t dim>
std::vector<typename Node<T, dim>::Id>* Node<T, dim>::getData() {
    return &data;
}
template <typename T, std::size_t dim>
Node<T, dim>* Node<T, dim>::operator[](std::size_t idx) {
    assert(idx < nbCells);
    return cells[idx];
}

// Build tree from DataPoints with a specified number of points by node
template <typename T, std::size_t dim>
bool Node<T, dim>::build(const DP& pts, std::size_t maxDataByNode, T maxSizeByNode, bool parallelBuild) {
    typedef typename PM::Vector Vector;

    // Build bounding box
    BoundingBox box;

    Vector minValues = pts.features.rowwise().minCoeff();
    Vector maxValues = pts.features.rowwise().maxCoeff();

    Point min = minValues.head(dim);
    Point max = maxValues.head(dim);

    Point radii = max - min;
    box.center = min + radii * 0.5;

    box.radius = radii(0);
    for (size_t i = 1; i < dim; ++i)
        if (box.radius < radii(i)) box.radius = radii(i);


    // Transform pts in data
    const size_t nbpts = pts.getNbPoints();
    std::vector<Id> indexes;
    indexes.reserve(nbpts);

    for (size_t i = 0; i < nbpts; ++i) indexes.emplace_back(Id(i));

    // build
    if (parallelBuild) {
        buildRecursiveParallel(pts, std::move(indexes), std::move(box), maxDataByNode, maxSizeByNode);
    } else {
        buildRecursive(pts, std::move(indexes), std::move(box), maxDataByNode, maxSizeByNode);
    }
    return true;
}

template <typename T, std::size_t dim>
bool Node<T, dim>::insert(const DP& newPts, std::size_t maxDataByNode, T maxSizeByNode, bool parallelInsert) {
    if (!isInsideBB(newPts)) {
        return false;
    }

    const size_t nbpts = newPts.getNbPoints();
    std::vector<Id> indexes;
    indexes.reserve(nbpts);

    #pragma omp parallel for
    for (size_t i = 0; i < nbpts; ++i) indexes.emplace_back(Id(i));

    if (parallelInsert) {
        insertRecursiveParallel(newPts, std::move(indexes), maxDataByNode, maxSizeByNode);
    } else {
        insertRecursive(newPts, std::move(indexes), maxDataByNode, maxSizeByNode);
    }
    return true;
}

template <typename T, std::size_t dim>
void Node<T, dim>::insertRecursive(const DP& newPts, std::vector<Id>&& dataToInsert, std::size_t maxDataByNode, T maxSizeByNode) {
    if (isLeaf()) {
            data.reserve(dataToInsert.size() + data.size());
            data.insert(data.end(), std::make_move_iterator(dataToInsert.begin()), make_move_iterator(dataToInsert.end()));

        if (((bb.radius * 2.0 <= maxSizeByNode) or (dataToInsert.size() <= maxDataByNode)) == false) {
            buildRecursive(newPts, std::move(data), BoundingBox{bb.center, bb.radius}, maxDataByNode, maxSizeByNode);
        }
        return;
    }

    // split datas
    const std::size_t nbData = dataToInsert.size();

	std::vector<Id> splittedData[nbCells];
    for (size_t i = 0; i < nbCells; ++i) splittedData[i].reserve(nbData);

    for (auto&& d : dataToInsert) (splittedData[idx(newPts, d)]).emplace_back(d);

    for (size_t i = 0; i < nbCells; ++i) splittedData[i].shrink_to_fit();

    for(int i = 0; i < nbCells; ++i) {
        this->cells[i]->insertRecursive(newPts, std::move(splittedData[i]), maxDataByNode, maxSizeByNode);
    }

    return;
}

template <typename T, std::size_t dim>
void Node<T, dim>::insertRecursiveParallel(const DP& newPts, std::vector<Id>&& dataToInsert, std::size_t maxDataByNode, T maxSizeByNode) {
    if (isLeaf()) {
            data.reserve(dataToInsert.size() + data.size());
            data.insert(data.end(), std::make_move_iterator(dataToInsert.begin()), make_move_iterator(dataToInsert.end()));

        if (((bb.radius * 2.0 <= maxSizeByNode) or (dataToInsert.size() <= maxDataByNode)) == false) {
            buildRecursive(newPts, std::move(data), BoundingBox{bb.center, bb.radius}, maxDataByNode, maxSizeByNode);
        }
        return;
    }

    // split dataToInsert
    const std::size_t nbData = dataToInsert.size();

	std::vector<Id> splittedData[nbCells];

    #pragma omp parallel for schedule(static, 1)
    for (size_t i = 0; i < nbCells; ++i) splittedData[i].reserve(nbData);

    for (auto&& d : dataToInsert) (splittedData[idx(newPts, d)]).emplace_back(d);

    #pragma omp parallel for schedule(static, 1)
    for (size_t i = 0; i < nbCells; ++i) splittedData[i].shrink_to_fit();

    #pragma omp parallel for schedule(static, 1)
    for(int i = 0; i < nbCells; ++i) {
        cells[i]->insertRecursive(newPts, std::move(splittedData[i]), maxDataByNode, maxSizeByNode);
    }

    return;
}

// Offset lookup table
template <typename T, std::size_t dim>
struct OctreeHelper;

template <typename T>
struct OctreeHelper<T, 3> {
    static const typename Node<T, 3>::Point offsetTable[Node<T, 3>::nbCells];
};
template <typename T>
const typename Node<T, 3>::Point OctreeHelper<T, 3>::offsetTable[Node<T, 3>::nbCells] = {{-0.5, -0.5, -0.5}, {+0.5, -0.5, -0.5}, {-0.5, +0.5, -0.5},
                                                                                               {+0.5, +0.5, -0.5}, {-0.5, -0.5, +0.5}, {+0.5, -0.5, +0.5},
                                                                                               {-0.5, +0.5, +0.5}, {+0.5, +0.5, +0.5}};

template <typename T>
struct OctreeHelper<T, 2> {
    static const typename Node<T, 2>::Point offsetTable[Node<T, 2>::nbCells];
};
template <typename T>
const typename Node<T, 2>::Point OctreeHelper<T, 2>::offsetTable[Node<T, 2>::nbCells] = {{-0.5, -0.5}, {+0.5, -0.5}, {-0.5, +0.5}, {+0.5, +0.5}};

template <typename T, std::size_t dim>
void Node<T, dim>::buildRecursive(const DP& pts, std::vector<Id>&& dataToBuild, BoundingBox&& bb, std::size_t maxDataByNode, T maxSizeByNode) {
    // Assign bounding box
    this->bb.center = bb.center;
    this->bb.radius = bb.radius;

    // Check stop condition
    if ((bb.radius * 2.0 <= maxSizeByNode) or (dataToBuild.size() <= maxDataByNode)) {
        // insert data
        data.insert(data.end(), std::make_move_iterator(dataToBuild.begin()), make_move_iterator(dataToBuild.end()));
        return;
    }

    // Split dataToBuild
	std::vector<Id> splittedData[nbCells];
    for (size_t i = 0; i < nbCells; ++i) splittedData[i].reserve(dataToBuild.size());

    for (auto&& d : dataToBuild) (splittedData[idx(pts, d)]).emplace_back(d);

    for (size_t i = 0; i < nbCells; ++i) splittedData[i].shrink_to_fit();

    // Compute new bounding boxes
    BoundingBox boxes[nbCells];
    const T half_radius = this->bb.radius * 0.5;
    for (size_t i = 0; i < nbCells; ++i) {
        const Point offset = OctreeHelper<T, dim>::offsetTable[i] * this->bb.radius;
        boxes[i].radius = half_radius;
        boxes[i].center = this->bb.center + offset;
    }

    // Build childs
    for (int i = 0; i < nbCells; ++i) {
        cells[i] = new Node<T, dim>(tree);
        cells[i]->buildRecursive(pts, std::move(splittedData[i]), std::move(boxes[i]), maxDataByNode, maxSizeByNode);
    }
}

template <typename T, std::size_t dim>
void Node<T, dim>::buildRecursiveParallel(const DP& pts, std::vector<Id>&& dataToBuild, BoundingBox&& bb, std::size_t maxDataByNode, T maxSizeByNode) {
    // Assign bounding box
    this->bb.center = bb.center;
    this->bb.radius = bb.radius;

    // Check stop condition
    if ((bb.radius * 2.0 <= maxSizeByNode) or (dataToBuild.size() <= maxDataByNode)) {
        // insert data
        data.insert(data.end(), std::make_move_iterator(dataToBuild.begin()), make_move_iterator(dataToBuild.end()));
        return;
    }

    // Split dataToBuild
	std::vector<Id> splittedData[nbCells];

    #pragma omp parallel for schedule(static, 1)
    for (size_t i = 0; i < nbCells; ++i) splittedData[i].reserve(dataToBuild.size());

    for (auto&& d : dataToBuild) (splittedData[idx(pts, d)]).emplace_back(d);

    #pragma omp parallel for schedule(static, 1)
    for (size_t i = 0; i < nbCells; ++i) splittedData[i].shrink_to_fit();

    // Compute new bounding boxes
    BoundingBox boxes[nbCells];
    const T half_radius = this->bb.radius * 0.5;

    #pragma omp parallel for schedule(static, 1)
    for (size_t i = 0; i < nbCells; ++i) {
        const Point offset = OctreeHelper<T, dim>::offsetTable[i] * this->bb.radius;
        const BoundingBox tmpBox = {this->bb.center + offset, half_radius};
        boxes[i] = tmpBox;
    }

    // Build childs
    #pragma omp parallel for schedule(static, 1)
    for (int i = 0; i < nbCells; ++i) {
        Node<T, dim>* child_i = new Node<T, dim>(tree);
        child_i->buildRecursive(pts, std::move(splittedData[i]), std::move(boxes[i]), maxDataByNode, maxSizeByNode);
        cells[i] = child_i;
    }
}

//------------------------------------------------------------------------------
template <typename T, std::size_t dim>
template <typename Callback>
bool Node<T, dim>::visit(Callback& cb) {
    // Call the callback for this node (if the callback returns false, then
    // stop traversing.
    if (!cb(*this)) return false;

    // If I'm a node, recursively traverse my children
    if (!isLeaf())
        for (size_t i = 0; i < nbCells; ++i)
            if (!cells[i]->visit(cb)) return false;

    return true;
}

