#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>

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
};

#endif
