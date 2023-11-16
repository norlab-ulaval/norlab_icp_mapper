#include <pointmatcher/PointMatcher.h>
#include "MapperModule.h"

//! Construct without parameter
MapperModule::MapperModule()
{}

typedef PointMatcher<float> PM;

//! Construct with parameters
MapperModule::MapperModule(const std::string& className, const PM::ParametersDoc paramsDoc, const PM::Parameters& params):
        PM::Parametrizable(className, paramsDoc, params)
{}

//! virtual destructor
MapperModule::~MapperModule()
{}

//! Init this filter
void MapperModule::init()
{}
