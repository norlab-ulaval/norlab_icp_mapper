#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include "norlab_icp_mapper/MapperModules/utils/octree.h"

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

int main() {
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
        
	PM::PointCloudGenerator pcg;
	DP pcl1 = pcg.generateUniformlySampledSphere(100.0, 10000000, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

	DP pcl2 = pcg.generateUniformlySampledSphere(150.0, 10000000, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

	// DP pcl1 = DP::load("../examples/data/scans/cloud_1690309709_285305600.vtk");
	// DP pcl2 = DP::load("../examples/data/scans/cloud_1690309709_385259008.vtk");

	std::size_t maxDataByNode = 1;
	float maxSizeByNode = 0.15;
	bool buildParallel = true;

	Octree<float> oc;
	{
		Timer("Build");
		oc.build(pcl1, maxDataByNode, maxSizeByNode, buildParallel);
	}
	{
		Timer("Insert");
		oc.insert(pcl2, maxDataByNode, maxSizeByNode, buildParallel);
	}
	oc.getCenter();
	return 0;	
}
