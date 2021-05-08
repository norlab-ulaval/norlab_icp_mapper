#include <fstream>
#include "CSVLogger.h"

void initializeCSVFile()
{
	std::ofstream file("/home/norlab/Desktop/mapping.csv");
	file << "t,x,y,z,qw,qx,qy,qz,mean_residual,T_icp,nb_points_added,nb_points_vicinity" << std::endl;
	file.close();
}

void logToCSV(const CSVLine& line)
{
	std::ofstream file("/home/norlab/Desktop/mapping.csv", std::ios::app);
	file << line.t.time_since_epoch().count() << ","
		 << line.x << ","
		 << line.y << ","
		 << line.z << ","
		 << line.qw << ","
		 << line.qx << ","
		 << line.qy << ","
		 << line.qz << ","
		 << line.residual << ","
		 << matrixToString(line.T_icp) << ","
		 << line.nbPointsAdded << ","
		 << line.nbPointsVicinity
		 << std::endl;
	file.close();
}

std::string matrixToString(const PM::TransformationParameters& matrix)
{
	std::string result = "[";
	for(int i = 0; i < matrix.rows(); i++)
	{
		result += "[";
		for(int j = 0; j < matrix.cols(); j++)
		{
			result += std::to_string(matrix(i, j));

			if(j < matrix.cols() - 1)
			{
				result += " ";
			}
		}
		result += "]";
	}
	result += "]";

	return result;
}
