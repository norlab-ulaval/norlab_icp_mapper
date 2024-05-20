#ifndef NORLAB_ICP_MAPPER_MAPPERMODULE_H
#define NORLAB_ICP_MAPPER_MAPPERMODULE_H

#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Parametrizable.h>

	//! A MapperModule creates a map from a different output and then updates the map with scan-pose pairs.
	/**
		The module might add information, remove dynamic points, or sample the map based on inter-point distance.
	*/
class MapperModule: public PointMatcher<float>::Parametrizable
	{
        typedef PointMatcher<float> PM;

		MapperModule();

public:
		//! Create a map from input point cloud.  This is the non-destructive version and returns a copy.
        virtual PM::DataPoints createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) = 0;

		//! Create a map from input point cloud without copying, modifying the input.
        virtual void inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) = 0;

		//! Update the map, adding points from the input. This is the non-destructive version and returns a copy.
		virtual PM::DataPoints updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose) = 0;

		//! Update the map, adding points from the input without copying.
		virtual void inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) = 0;
    protected:
        MapperModule(const std::string& className, const PM::ParametersDoc paramsDoc, const PM::Parameters& params);
        ~MapperModule() override;
    };

#endif //NORLAB_ICP_MAPPER_MAPPERMODULE_H
