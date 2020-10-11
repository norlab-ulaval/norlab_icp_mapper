#ifndef NORLAB_ICP_MAPPER_STATE_H
#define NORLAB_ICP_MAPPER_STATE_H

namespace norlab_icp_mapper
{
	typedef float T;
	
	typedef struct
	{
		Eigen::Matrix<T, 3, 1> position;
		Eigen::Matrix<T, 3, 3> positionCovariance;
		std::chrono::time_point<std::chrono::steady_clock> positionTimeStamp;
		Eigen::Matrix<T, 4, 1> orientation;
		Eigen::Matrix<T, 3, 3> orientationCovariance;
		std::chrono::time_point<std::chrono::steady_clock> orientationTimeStamp;
		Eigen::Matrix<T, 3, 1> linearAcceleration;
		Eigen::Matrix<T, 3, 3> linearAccelerationCovariance;
		std::chrono::time_point<std::chrono::steady_clock> linearAccelerationTimeStamp;
		Eigen::Matrix<T, 3, 1> angularVelocity;
		Eigen::Matrix<T, 3, 3> angularVelocityCovariance;
		std::chrono::time_point<std::chrono::steady_clock> angularVelocityTimeStamp;
	} State;
}

#endif
