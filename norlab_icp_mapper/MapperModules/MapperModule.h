#ifndef NORLAB_ICP_MAPPER_MAPPERMODULE_H
#define NORLAB_ICP_MAPPER_MAPPERMODULE_H

#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Parametrizable.h>

	//! TODO
	/**
		TODO The filter might add information, for instance surface normals, or might change the number of points, for instance by randomly removing some of them.
	*/
class MapperModule: public PointMatcher<float>::Parametrizable
	{
        typedef PointMatcher<float> PM;

		MapperModule();
        virtual void init();

    public:
        virtual PM::DataPoints createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) = 0;
        virtual void inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) = 0;

		//! Apply filters to input point cloud.  This is the non-destructive version and returns a copy.
		virtual PM::DataPoints updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose) = 0;

		//! Apply these filters to a point cloud without copying.
		virtual void inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) = 0;
    protected:
        MapperModule(const std::string& className, const PM::ParametersDoc paramsDoc, const PM::Parameters& params);
        ~MapperModule() override;
    };

#endif //NORLAB_ICP_MAPPER_MAPPERMODULE_H
