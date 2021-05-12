#ifndef NORLAB_ICP_MAPPER_CSVLOGGER_H
#define NORLAB_ICP_MAPPER_CSVLOGGER_H

#include <pointmatcher/PointMatcher.h>
#include <chrono>

typedef PointMatcher<float> PM;

typedef struct CSVLine
{
	std::chrono::time_point<std::chrono::steady_clock> t;
	float x;
	float y;
	float z;
	float qw;
	float qx;
	float qy;
	float qz;
	float residual;
	PM::TransformationParameters T_icp;
	int nbPointsAdded;
	int nbPointsVicinity;
} CSVLine;

void initializeCSVFile(const std::string& csvFileName);

std::string matrixToString(const PM::TransformationParameters& matrix);

void logToCSV(const CSVLine& line, const std::string& csvFileName);

#endif
