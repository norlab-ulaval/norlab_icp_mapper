# Creating a MapperModule

In the following tutorials we will discuss how you can extend the functionality provided in `norlab_icp_mapper` by taking advantage of its modular design.
The following tutorial will detail the development of a mapper module, which is not yet included in `norlab_icp_mapper`.

## Generic procedure
To implement a _Dummy_ filter that adds all new points into an existing map, we have to follow these steps:

1. Create files in the [norlab_icp_mapper/MapperModules/](https://github.com/norlab-ulaval/norlab_icp_mapper/tree/master/norlab_icp_mapper/MapperModules) folder
	- the header : `Dummy.h`
	- the implementation file : `Dummy.cpp`

1. Declare your filter in the header with the minimal following interface:
```C++
#ifndef NORLAB_ICP_MAPPER_DUMMYMAPPERMODULE_H
#define NORLAB_ICP_MAPPER_DUMMYMAPPERMODULE_H

#include "MapperModule.h"
class DummyMapperModule : public MapperModule
{
    typedef PointMatcher<float> PM;
public:
    typedef typename PM::DataPoints DataPoints;
    typedef typename PM::TransformationParameters TfParams;
    
    inline static const std::string description()
    {
        return "description";
    }

    inline static const PM::ParametersDoc availableParameters()
    {
        return {
                {"param", "param desc", "default value"}
        };
    }

    explicit DummyMapperModule(const PM::Parameters& params = PM::Parameters());
    ~DummyMapperModule() override = default;

    DataPoints createMap(const DataPoints& input, const TfParams& pose) override;
    void inPlaceCreateMap(DataPoints& input, const TfParams& pose) override;

    DataPoints updateMap(const DataPoints& input, const DataPoints& map, const TfParams& pose) override;
    void inPlaceUpdateMap(const DataPoints& input, DataPoints& map, const TfParams& pose) override;
};
#endif //NORLAB_ICP_MAPPER_DUMMYMAPPERMODULE_H
```
3. Implement the filter in the `.cpp` file and declare the template at the end of the file. Note that the map and input are simply concatenated since the input is already in the map frame when it enters the updateMap and inPlaceUpdateMap methods.
    ```c++
    #include "Dummy.h"
    
    using DataPoints = DummyMapperModule::DataPoints;
    using TfParams = DummyMapperModule::TfParams;

    DummyMapperModule::DummyMapperModule(const PM::Parameters& params):
        MapperModule("DummyMapperModule", DummyMapperModule::availableParameters(), params)
        {}
    
    DataPoints DummyMapperModule::createMap(const DataPoints& input, const TfParams& pose)
    {
        DataPoints outputMap(input);
        inPlaceCreateMap(outputMap, pose);
        return outputMap;
    }
    
    void DummyMapperModule::inPlaceCreateMap(DataPoints& input, const TfParams& pose)
    {
    // keep the input untouched
    }
    
    DataPoints DummyMapperModule::updateMap(const DataPoints& input, const DataPoints& map, const TfParams& pose)
    {
        DataPoints outputMap(map);
        inPlaceUpdateMap(input, outputMap, pose);
        return outputMap;
    }
    
    void DummyMapperModule::inPlaceUpdateMap(const DataPoints& input, DataPoints& map, const TfParams& pose)
    {
        map.concatenate(input);
    }
    ```
   
4. Add it to the _Registry_ in [norlab_icp_mapper/Mapper.cpp](https://github.com/norlab-ulaval/norlab_icp_mapper/blob/master/norlab_icp_mapper/Mapper.cpp)
    - Add the file to header imports: `#include "MapperModules/Dummy.h"`
	- If the filter has some parameters:
   ```c++
   ADD_TO_REGISTRAR(MapperModule, DummyMapperModule, DummyMapperModule)
   ``` 
    - If not:
   ```c++
   ADD_TO_REGISTRAR_NO_PARAM(MapperModule, DummyMapperModule, DummyMapperModule)
   ```
6. Finally, add the source file in the [CMakeLists.txt](https://github.com/norlab-ulaval/norlab_icp_mapper/blob/master/CMakeLists.txt):
    - In the `MapperModules` section of `add_library(...`.
    - In the `install(FILES...` section defining files to install.

Now recompile the library and check that the new mapper module can be accessed.

## Writing Python bindings
Every Mapper module should be covered with a Python binding, located in [python/src/MapperModules](https://github.com/norlab-ulaval/norlab_icp_mapper/blob/master/python/src/MapperModules).
We will now create Python bindings for our recently written `DummyMapperModule`.

1. First of all, we need to install our recent changes
    ```shell
    cd build
    sudo make install
    ```
2. Create files in the [python/src/MapperModules](https://github.com/norlab-ulaval/norlab_icp_mapper/tree/master/norlab_icp_mapper/MapperModules) folder
	- the header : `dummy.h`
	- the implementation file : `dummy.cpp`

3. Declare your filter in the header with the minimal following interface:
```C++
#ifndef NORLAB_ICP_MAPPER_DUMMY_H
#define NORLAB_ICP_MAPPER_DUMMY_H

#include "../norlab_icp_mapper_helper.h"

namespace python
{
    namespace mappermodules
    {
        void pybindDummy(py::module& p_module);
    }
}

#endif //NORLAB_ICP_MAPPER_DUMMY_H

```
4. Implement the filter in the `.cpp` file and declare the template at the end of the file.
    ```c++
    #include "dummy.h"
    #include "norlab_icp_mapper/MapperModules/Dummy.h"
    
    namespace python
    {
        namespace mappermodules
        {
            void pybindDummy(py::module& p_module)
            {
                py::class_<DummyMapperModule, std::shared_ptr<DummyMapperModule>, MapperModule>(p_module, "DummyMapperModule")
                        .def_static("description", &DummyMapperModule::description)
                        .def_static("availableParameters", &DummyMapperModule::availableParameters)
    
                        .def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")
    
                        .def("createMap", &DummyMapperModule::createMap, py::arg("input"), py::arg("pose"))
                        .def("inPlaceCreateMap", &DummyMapperModule::inPlaceCreateMap, py::arg("input"), py::arg("pose"))
                        .def("updateMap", &DummyMapperModule::updateMap, py::arg("input"), py::arg("map"), py::arg("pose"))
                        .def("inPlaceUpdateMap", &DummyMapperModule::inPlaceUpdateMap, py::arg("input"), py::arg("map"), py::arg("pose"));
            }
        }
    }
    ```
   
5. Add it to the _mapperModules_ Pybind11 module in [python/src/norlab_icp_mapper.cpp](https://github.com/norlab-ulaval/norlab_icp_mapper/blob/master/python/src/norlab_icp_mapper.cpp)
    - Add the file to header imports: `#include "mappermodules/dummy.h"`
	- Call the pybind function with the `mapperModulesModule`: 
   ```c++
   python::mappermodules::pybindDummy(mapperModulesModule);
   ```
6. Finally, add the source file in the [python/CMakeLists.txt](https://github.com/norlab-ulaval/norlab_icp_mapper/blob/master/CMakeLists.txt), when setting the `PYBIND11_SOURCES` variable:
```cmake
set(PYBIND11_SOURCES
    ...
    src/mappermodules/dummy.cpp
)
```

You can now install your changes with (from `python` directory):
```shell
pip install .
```