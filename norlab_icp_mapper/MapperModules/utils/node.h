#pragma once

#include <vector>
#include <pointmatcher/PointMatcher.h>
#include <eigen3/Eigen/Core>

template <class T>
inline constexpr T pow(const T base, const std::size_t exponent) {
    return exponent == 0 ? 1 : base * pow(base, exponent - 1);
}

template <typename T, std::size_t dim>
class Octree_;

template <typename T, std::size_t dim>
class Node_ {
   public:
    using PM = PointMatcher<T>;
    using DP = typename PM::DataPoints;
    using Id = typename DP::Index;

    using Point = Eigen::Matrix<T, dim, 1>;

    static constexpr std::size_t nbCells = pow(2, dim);

   private:
    struct BoundingBox {
        Point center;
        T radius;
    };

    Node_* cells[nbCells];

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

    BoundingBox bb;

    std::vector<Id> data;

    Octree_<T, dim>* tree;

    int depth;

   public:
    Node_(Octree_<T, dim>* tree);
    Node_(const Node_<T, dim>& o);  // Deep-copy

    virtual ~Node_();

    Node_<T, dim>& operator=(const Node_<T, dim>& o);  // Deep-copy

    bool isLeaf() const;
    bool isEmpty() const;
    bool isInsideBB(const DP& pts) const;
    int getDepth() const;

    inline std::size_t idx(const Point& pt) const;
    inline std::size_t idx(const DP& pts, const Id d) const;

    T getRadius() const;
    Point getCenter() const;

    std::vector<Id>* getData();
    Node_<T, dim>* operator[](std::size_t idx);

    bool build(const DP& pts, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.), bool parallelBuild = false);
    bool insert(const DP& newPts, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.), bool parallelInsert = false);

   protected:
    void buildRecursive(const DP& pts, std::vector<Id>&& dataToBuild, BoundingBox&& bb, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.));
    void buildRecursiveParallel(const DP& pts, std::vector<Id>&& dataToBuild, BoundingBox&& bb, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.));

    void insertRecursive(const DP& newPts, std::vector<Id>&& dataToInsert, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.));
    void insertRecursiveParallel(const DP& newPts, std::vector<Id>&& dataToInsert, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.));

    void subsample(int nbOfPointsToDelete);

   public:
    template <typename Callback>
    bool visit(Callback& cb);

    std::vector<Node_<T, dim>*> getLeaves() const;
};

template <typename T>
using QuadtreeNode = Node_<T, 2>;
template <typename T>
using OctreeNode = Node_<T, 3>;

#include "node.hpp"
