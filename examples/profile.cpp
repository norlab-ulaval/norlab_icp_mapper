#include <iostream>
#include <fstream>
#include <pointmatcher/PointMatcher.h>
#include "norlab_icp_mapper/MapperModules/utils/octree.h"
#include "norlab_icp_mapper/MapperModules/utils/node.h"

class Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    const std::string name;

public:
    Timer(const std::string& name) : name(name) { startTime = std::chrono::high_resolution_clock::now(); }

    double getElapsedMicroseconds() {
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = endTime - startTime;

        return elapsed.count();
    }

    ~Timer() { std::cout << name << " took " << getElapsedMicroseconds() << "ms" << std::endl; }
};

void export_leaves(const std::vector<OctreeNode<float>*>& leaves) {
	std::ofstream file;
	file.open("feuille.json");
	std::cout << leaves.size() << "\n";
	file << "[\n";
	for (OctreeNode<float>* leave : leaves) {
		file << "  {\n";
		file << "    \"depth\": " << leave->getDepth() << ",\n";
		file << "    \"center\": {\n";
		file << "      \"x\": " << leave->getCenter().x() << ",\n";
		file << "      \"y\": " << leave->getCenter().y() << ",\n";
		file << "      \"z\": " << leave->getCenter().z() << "\n";
		file << "    },\n";
		file << "    \"radius\": " << leave->getRadius() << "\n";
		file << "  },\n";
	}
	file << "]\n";
	file.close();
}

int main() {
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
        
	PM::PointCloudGenerator pcg;
	// DP pcl1 = pcg.generateUniformlySampledSphere(100.0, 10000000, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
	//
	// DP pcl2 = pcg.generateUniformlySampledSphere(150.0, 10000000, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

	DP pcl1 = DP::load("../examples/data/scans/cloud_1690309709_285305600.vtk");
	DP pcl2 = DP::load("../examples/data/scans/cloud_1690309709_385259008.vtk");

	std::size_t maxDataByNode = 15;
	float maxSizeByNode = 1.0;
	bool buildParallel = true;

	Octree<float> oc(maxDataByNode, maxSizeByNode, buildParallel);
	{
		Timer titi = Timer("Build");
		oc.build(pcl1);
	}
	{
		Timer titi = Timer("Insert");
		oc.insert(pcl2);
	}
	export_leaves(oc.getLeaves());
	
	std::vector<Octree<float>::Id> deletedData = oc.getDeletedData();

	for (Octree<float>::Id id : deletedData) {
		std::cout << id << std::endl;
	}
}
