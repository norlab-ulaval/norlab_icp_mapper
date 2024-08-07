#include "OctreeMapperModule.h"

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

OctreeMapperModule::OctreeMapperModule(const PM::Parameters& params) : MapperModule("OctreeMapperModule", OctreeMapperModule::availableParameters(), params) {
    // This is necessary otherwise an InvalidParameter is thrown from libpointmatcher
    // as it detects that a set parameter was not accessed.
    buildParallel = PM::Parametrizable::get<bool>("buildParallel");
    PM::Parametrizable::get<std::size_t>("samplingMethod");
    maxPointByNode =  PM::Parametrizable::get<std::size_t>("maxPointByNode");
    maxSizeByNode = PM::Parametrizable::get<float>("maxSizeByNode");
    octree = Octree<float>();
}

PointMatcher<float>::DataPoints OctreeMapperModule::createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) {
    DataPoints outputMap(input);
    inPlaceCreateMap(outputMap, pose);
    return outputMap;
}

void OctreeMapperModule::inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) {
    DataPoints emptyMap = input.createSimilarEmpty();
    inPlaceUpdateMap(emptyMap, input, pose);
}

PointMatcher<float>::DataPoints OctreeMapperModule::updateMap(const PM::DataPoints& input, const PM::DataPoints& map,
                                                              const PM::TransformationParameters& pose) {
    DataPoints outputMap(map);
    inPlaceUpdateMap(input, outputMap, pose);
    return outputMap;
}

void OctreeMapperModule::inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) {
    map.concatenate(input);
    if (!octree_is_built) {
        {
            Timer("-- Build for pcl #" + std::to_string(i));
            octree.build(map, maxPointByNode, maxSizeByNode, buildParallel);
        }
        octree_is_built = true;
    } else {
        bool insert_worked = true;
        {
            Timer("-- Insert for pcl #" + std::to_string(i));
            insert_worked = octree.insert(input, maxPointByNode, maxSizeByNode, buildParallel);
        }
        if (!insert_worked) {
            {
                Timer("-- Build cuz cant insert for pcl #" + std::to_string(i));
                octree.clearTree();
                octree.build(map, maxPointByNode, maxSizeByNode, buildParallel);
            }
        }
    }
    // {
    //     oc.build(map, maxPointByNode, maxSizeByNode, buildParallel);
    // }

    i++;
}
