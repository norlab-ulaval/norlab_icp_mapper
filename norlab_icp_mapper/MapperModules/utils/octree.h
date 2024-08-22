#pragma once

#include <vector>
#include <pointmatcher/PointMatcher.h>
#include "node.h"
#include <eigen3/Eigen/Core>
#include "MapperModules/utils/node.h"

template <typename T, std::size_t dim>
class Octree_ {
   public:
    using PM = PointMatcher<T>;
    using DP = typename PM::DataPoints;
    using Id = typename DP::Index;

    using DataContainer = std::vector<Id>;

    using Point = Eigen::Matrix<T, dim, 1>;

   private:
    /******************************************************
     *	Cells id are assigned as their position
     *   from the center (+ greater than center, - lower than center)
     *
     *		for 3D case									for 2D case
     *
     *	  	0	1	2	3	4	5	6	7		  	0	1	2	3
     * 	x:	-	+	-	+	-	+	-	+		x:	-	+	-	+
     * 	y:	-	-	+	+	-	-	+	+		y:	-	-	+	+
     * 	z:	-	-	-	-	+	+	+	+
     *
     *****************************************************/


    Node<T, dim>* root;

    size_t maxDataByNode;
    T maxSizeByNode;
    bool runInParallel;

    std::vector<Id> deletedDataFromLastModification;

   public:
    Octree_(std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.), bool runInParallel = false);

    virtual ~Octree_();

    bool build(const DP& pts);
    bool insert(const DP& newPts);

    void registerDeletedData(const std::vector<Id>& deletedData);

   public:
    template <typename Callback>
    bool visit(Callback& cb);
};

#include "octree.hpp"

template <typename T>
using Quadtree = Octree_<T, 2>;
template <typename T>
using Octree = Octree_<T, 3>;
