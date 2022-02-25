#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>
#include <chrono>

class Trajectory
{
public:
	Trajectory(int dimension);
	void addPoint(Eigen::VectorXf point);
	void save(std::string filename) const;
	void clearPoints();

private:
	int dimension;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> points;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> timestamps;
	bool startTimeSet = false;
	std::chrono::high_resolution_clock::time_point start;
};

#endif
