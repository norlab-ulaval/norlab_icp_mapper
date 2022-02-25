#include "Trajectory.h"
#include <pointmatcher/PointMatcher.h>

Trajectory::Trajectory(int dimension):
		dimension(dimension)
{
}

void Trajectory::addPoint(Eigen::VectorXf point)
{
	points.conservativeResize(dimension, points.cols() + 1);
	points.col(points.cols() - 1) = point;

	timestamps.conservativeResize(1, timestamps.cols() + 1);


	float relativeTime;
	if(!startTimeSet)
	{
		startTimeSet = true;
		start = std::chrono::high_resolution_clock::now();
		relativeTime = 0.0;
	}
	else
	{
		auto now = std::chrono::high_resolution_clock::now();
		auto dif = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
		relativeTime = (float) dif.count() / 1000.0;
	}
	timestamps(timestamps.cols() - 1) = relativeTime;
}

void Trajectory::save(std::string filename) const
{
	PointMatcher<float>::DataPoints trajectory;
	trajectory.addFeature("x", points.row(0));
	trajectory.addFeature("y", points.row(1));
	if(dimension == 3)
	{
		trajectory.addFeature("z", points.row(2));
	}
	trajectory.addDescriptor("timestamp", timestamps);
	trajectory.save(filename);
}

void Trajectory::clearPoints()
{
	points.conservativeResize(0, 0);
}
