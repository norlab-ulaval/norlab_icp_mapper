#pragma once

#include <vector>
#include <pointmatcher/PointMatcher.h>
#include <eigen3/Eigen/Core>

template <class T>
inline constexpr T pow(const T base, const std::size_t exponent) {
    return exponent == 0 ? 1 : base * pow(base, exponent - 1);
}

template <typename T, std::size_t dim>
class Node {
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

    Node* cells[nbCells];

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

    std::size_t depth;

    size_t maxDataByNode;
    T maxSizeByNode;
    bool isRoot;


   public:
    Node(bool isRoot = true);
    Node(const Node<T, dim>& o);  // Deep-copy
    Node(Node<T, dim>&& o);

    virtual ~Node();

    Node<T, dim>& operator=(const Node<T, dim>& o);  // Deep-copy
    Node<T, dim>& operator=(Node<T, dim>&& o);

    bool isLeaf() const;
    bool isEmpty() const;
    bool getIsRoot() const;
    bool isInsideBB(const DP& pts) const;

    inline std::size_t idx(const Point& pt) const;
    inline std::size_t idx(const DP& pts, const Id d) const;

    std::size_t getDepth() const;

    T getRadius() const;
    Point getCenter() const;

    std::vector<Id>* getData();
    Node<T, dim>* operator[](std::size_t idx);

    bool build(const DP& pts, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.), bool parallelBuild = false);
    bool insert(const DP& newPts, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.), bool parallelInsert = false);

    void clearTree();


   protected:
    void buildRecursive(const DP& pts, std::vector<Id>&& dataToBuild, BoundingBox&& bb, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.));
    void buildRecursiveParallel(const DP& pts, std::vector<Id>&& dataToBuild, BoundingBox&& bb, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.));

    void insertRecursive(const DP& newPts, std::vector<Id>&& dataToInsert, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.));
    void insertRecursiveParallel(const DP& newPts, std::vector<Id>&& dataToInsert, std::size_t maxDataByNode = 1, T maxSizeByNode = T(0.));


   public:
    template <typename Callback>
    bool visit(Callback& cb);
};

#include "node.hpp"
