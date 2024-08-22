#include "OctreeMapperModule.h"
#include <fstream>

class Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    const std::string name;
    std::ofstream* file;

public:
    Timer(const std::string& name, std::ofstream* file) : name(name), file(file) { startTime = std::chrono::high_resolution_clock::now(); }

    double getElapsedMicroseconds() {
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = endTime - startTime;

        return elapsed.count();
    }

    ~Timer() { *file << name << " took " << getElapsedMicroseconds() << "ms" << std::endl; }
};

OctreeMapperModule::OctreeMapperModule(const PM::Parameters& params) : MapperModule("OctreeMapperModule", OctreeMapperModule::availableParameters(), params) {
    // This is necessary otherwise an InvalidParameter is thrown from libpointmatcher
    // as it detects that a set parameter was not accessed.
    buildParallel = PM::Parametrizable::get<bool>("buildParallel");
    PM::Parametrizable::get<std::size_t>("samplingMethod");
    maxPointByNode =  PM::Parametrizable::get<std::size_t>("maxPointByNode");
    maxSizeByNode = PM::Parametrizable::get<float>("maxSizeByNode");
    octree = Octree<float>(maxPointByNode, maxSizeByNode, buildParallel);
    file = std::ofstream();
    file.open("/home/nicolaslauzon/urmom-build.txt");
}

OctreeMapperModule::~OctreeMapperModule() {
    file.close();
    std::cout << "Closed file" << std::endl;
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
            Timer titi = Timer("-- Insert pcl #" + std::to_string(i), &file);
            octree.build(map);
        }
        octree_is_built = true;
    } else {
        bool insert_worked = true;
        {
            Timer titi = Timer("-- Insert pcl #" + std::to_string(i), &file);
            insert_worked = octree.insert(input);
        }
        if (!insert_worked) {
            {
                Timer titi = Timer("-- Insert pcl #" + std::to_string(i), &file);
                octree = Octree<float>(maxPointByNode, maxSizeByNode, buildParallel);
                octree.build(map);
            }
        }
    }
    Octree<float> oc;
    {
        Timer titi = Timer("-- Build pcl #" + std::to_string(i), &file);
        oc.build(map);
    }

    i++;
}
