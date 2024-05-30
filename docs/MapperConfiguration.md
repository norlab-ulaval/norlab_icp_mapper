# Mapper Configuration with YAML

## Overview

The mapper is configured with yaml files which provide excellent modularity.
The implementation is based on [libpointmatcher](https://github.com/norlab-ulaval/libpointmatcher) and relies on the [yaml-cpp](https://github.com/jbeder/yaml-cpp/) library.
A basic structure of a configuration file looks like this:

```yaml
mapper:
  ...
input:
  ...
icp:
  ...
post:
  ...
```
The four keys correspond to different stages of the mapping pipeline.
If either of them is not specified, default values that work reasonably well in more cases will be used instead.
In the next sections, we will describe the keys with their possible values one by one.

## Configuring the Mapper
A typical `mapper` configuration can look like this:
```yaml
mapper:
  updateCondition:
    type: distance
    value: 1.0
    
  sensorMaxRange: 200

  mapperModule:
    - ComputeDynamicsMapperModule:
        thresholdDynamic: 0.9
        alpha: 0.8
        beta: 0.99
        beamHalfAngle: 0.01
        epsilonA: 0.01
        epsilonD: 0.01

    - OctreeMapperModule:
        buildParallel: 1
        maxSizeByNode: 0.15
        samplingMethod: 1
```
The `mapper` key contain the following values:

- **updateCondition**: When to merge new scan into an existing map.
- **sensorMaxRange**: What is the maximum reliable range of the current sensor.
- **mapperModule**: Various filters and algorithms applied on the scan+map pair during each update.

Let's take a look at them one by one:

### Mapper update condition
The update condition defines when a new scan is accepted to be merged into an existing map.
Modern depth sensors usually produce tens or even hundreds of thousands of points at high frequencies, sometimes over 10 Hz.
Merging all these points into the map would be computational intensive, as lots of information in the scans is in fact redundant.
The possible types of the update condition are:

- **distance** [0.0-inf): Euclidean distance between the last pose when a map update happened and the current pose.  
- **delay** [0.0-inf): Time delay between the timestamp when a map update happened and the current scan timestamp.
- **overlap** [0.0-1.0]: Overlap between the current sensor scan and map points.

In brackets are the possible values for each update condition.

!!! warning

    A combination of multiple update conditions is currently not supported.

#### Maximum sensor range
Most sensors have a maximum range defined in their datasheet.
Values can still occur pass this value, however such returns are prone to large error and should be filtered out.
This `sensorMaxRange` values defines a radiusFilter that removes all points from a scan that is outside of a sphere centered at the robot.

Last but not least, the `sensorMaxRange` also defines an area around the robot where the Voxel Manager operates.

##### Voxel Manager
The purpose of the voxel manager is to limit the computational complexity of the map maintenance, instead of letting it
grow with its size.
A map is divided into voxels and only these cells that are close to the robot are kept in RAM.
The remaining points are stored on the system's hard drive.
The local map dimensions are defined as twice the `sensorMaxRange` plus a margin of two cells on each side.
A single cell size is currently hard-coded to 20 meters.
As an example, setting the `sensorMaxRange` to 100 meters, the local map stored on the RAM will be a cube with a side of
$$
2 \times 100 + 2 \times (2 \times 20)~m = 280~m
$$
with the robot located at its center.
For more details on the long-term map storage, check the [Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned](https://norlab.ulaval.ca/publications/field-report-ltr/) paper.

### Mapper Modules
Mapper Modules are a way to configure what happens when a lidar scan is merged into the map.
This can include an examination of new points already exist in the map, or removal of dynamic points.

The library implements these filters:
#### Down-sampling

_PointDistance_

Adds a new descriptor to an existing point cloud or overwrites existing descriptor with the same name.

- **Impact on the number of points:** reduces number of points 

| Parameter       | Description                                                                          | Default value | Allowable range |
|-----------------|:-------------------------------------------------------------------------------------|:--------------|:----------------|
| minDistNewPoint | Minimum distance between a new point and its nearest neighbor in the map (in meters) | 0.03          | [0.0, inf)      |

_Octree_

The filter use the efficent spatial representation of the pointcloud by the octree to sub-sample point in each leaf.
Since the library just provides a wrapper for `libpointmatcher`, check [its docs](https://libpointmatcher.readthedocs.io/en/latest/DataFilters/#octree-grid-filter) for more details.

#### Dynamic points management
_ComputeDynamics_

Computes a per-point value defining the probability that the points are dynamic, originating from, e.g., walking pedestrians, or falling snow.
The dynamic filter allows longer operations as dynamic points are not accumulating in the map.
To identify the dynamic points, ray tracing is used.
If an incoming scan point is located behind a map point, the probability of this map point being dynamic is increased.
If this probability surpasses the predefined threshold, it is effectively removed from the map.
To limit computation time for this filter, map points located further than `sensorMaxRange` do not enter the filtering process.


- **Required descriptors:**  probabilityDynamic, normals
- **Modified descriptor:** probabilityDynamic
- **Sensor assumed to be at the origin:** no  
- **Impact on the number of points:** reduces number of points

The probabilityDynamic descriptor should be added to the input point cloud like this:
```yaml
input:
  - AddDescriptorDataPointsFilter:
      descriptorName: probabilityDynamic 
      descriptorDimension: 1
      descriptorValues: [0.6] # This value is the initial probability of each point being dynamic
```

| Parameter        | Description                                                        | Default value | Allowable range |
|------------------|:-------------------------------------------------------------------|:--------------|:----------------|
| thresholdDynamic | Probability at which a point is considered permanently dynamic.    | 0.6           | [0.0, 1.0]      |
| alpha            | Probability of staying static given that the point was static.     | 0.8           | [0.0, 1.0]      |
| beta             | Probability of staying dynamic given that the point was dynamic.   | 0.99          | [0.0, 1.0]      |
| beamHalfAngle    | Half angle of the cones formed by the sensor laser beams (in rad). | 0.01          | [0.0, pi]       |
| epsilonA         | Error proportional to the sensor distance.                         | 0.01          | [0.0, 1.0]      |
| epsilonD         | Fix error on the sensor distance (in meters)                       | 0.01          | [0.0, inf)      |
| sensorMaxRange   | aximum reading distance of the laser (in meters).                  | 200           | [0.0, inf)      |

!!! tip "Development"

    Check the [developer's guide](MapperModuleDev.md) to start writing your own Mapper Module.

### Configuring the Input filters
The input filters dictate what filters are applied to the scan point cloud before being processed.
For example, to randomly remove 70 % of the input points, use this code to your yaml configuration:
```yaml
input:
  - RandomSamplingDataPointsFilter:
      prob: 0.3
      randomSamplingMethod: 1
      seed: 0
```

The list of filters can contain any DataPointsFilter accepted by `libpointmatcher`.
See the exhaustive list [here](https://libpointmatcher.readthedocs.io/en/latest/DataFilters/).

## Configuring the ICP
This key-value pair contains the ICP parameters used by `libpointmatcher` to do the registration of new point clouds in the map.
For example, in order to use a KDTreeMatcher with 6 nearest neighbors and a PointToPlane error minimizer, use
```yaml
icp:
  matcher:
    KDTreeMatcher:
      knn: 6
      maxDist: 2.0
      epsilon: 1
      
  errorMinimizer:
    PointToPlaneErrorMinimizer:
```
Similarly to the Input filters configuration, the `icp` config can also contain various DataPointsFilters, defined under the `readingDataPointsFilters` and `referenceDataPointsFilters` keys.
See [this guide](https://libpointmatcher.readthedocs.io/en/latest/ICPIntro/) for a more detailed explanation on how to configure this section.
### Configuring the Post filters
Post filters are applied to the map after adding the new point cloud.
A good example of a filter that is typically calculated post-merge are the `SurfaceNormalDataPointsFilter` and `CutAtDescriptorThresholdDataPointsFilter`.
The `SurfaceNormalDataPointsFilter` calculates a surface normal for every point in the map.
Since the normals can change after updating the map with a new scan, doing this operation post-merge makes a good sense.
The `CutAtDescriptorThresholdDataPointsFilter` removes all points that contain a descriptor value higher that the set threshold.
Here we use it to remove dynamic points that were previously calculated with the `ComputeDynamicsMapperModule`. 
```yaml
post:
    - SurfaceNormalDataPointsFilter:
        knn: 15
    
    - CutAtDescriptorThresholdDataPointsFilter:
        descName: probabilityDynamic
        useLargerThan: 1
        threshold: 0.8
```
Similarly to the input filters, a full list of all possible filters can be found [here](https://libpointmatcher.readthedocs.io/en/latest/DataFilters/).