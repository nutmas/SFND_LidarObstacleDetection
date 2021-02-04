# Sensor Fusion Sensor Fusion
### Lidar Obstacle Detection Project

<img src="media/ObstacleDetection.gif" width="612" height="392" />

## Overview
C++ development of a pipeline to process 3D Point Cloud Data(PCD) collected by a lidar sensor. Implementation of various algorithms allows the PCD to be classified into road surface and obstacles. The obstacles are encompassed in bounding boxes to ease visualisation.

---

## Installation steps

To run this code the following installation steps are required:

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/nutmas/SFND_LidarObstacleDetection.git


```

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```



---

## Other Important Dependencies

* cmake >= 3.14
* make >= 3.8 
* gcc/g++ >= 5.4
* PCL >= 1.2

---

## Build the code

1. Access the folder created from cloning `cd SFND_LidarObstacleDetection`
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 

---


## Usage

After completing the installation steps the model can be trained by performing the following step:

1. From terminal window; change to build folder of project `cd SFND_LidarOstacleDetection/build`
2. ./environment

---


## Model Documentation

Hyper-parameters to configure the algorithms are contained in the main program `environment.cpp`.


The four main steps of the pipeline:

**Cloud Filtering:** 
A whole frame of PCD is passed to the filter stage. The data is downsampled to voxel cubes. The data is also cropped to only focus on the region of interest. This region and the voxel size are defined in the hyper-parameters. A roof defined region is also subtracted from the data.

**Cloud Seperation:**
The filtered PCD is separated using the RANSAC algorithm. Hyper-parameters defining the max iterations and point distance threshold for the algorithm. Points are fitted to a best fit plane, which can represent the road plane. The cloud is separated into a pair of clouds, one to hold the points of the road and one to hold the non-road points.

**Cloud Clustering:**
Euclidean clustering algorithm is utilised to process the non-road points pointcloud. The algorithm finds nearest points to from clusters. Recursively traversing a KDTree is used to support the point searching. Smaller clustered clouds are generated to represent the different obstacles in the scene. The clusters are defined by minimum size, maximum size and maximum distance between points to create a cluster. the hyper-parameters configure these values.

**Visualisation:**
Using the min and max dimensions of each point cloud a bounding box is created to support the visualisation of each cluster object. Using render flags, the difference pointclouds throughout the pipeline can be visualised, along with the bounding boxes.

---

## Reflection

This course explored utilising the PointCloud Library to perform the cloud separation and clustering and also how to create your own version. The project task was to implement these created methods into real PCD.

During the development the processing time could be influenced by avoiding replicating variables. Passing pointcloud data in an optimal way and working on a single instance is extremely important to avoid very large data processing times. 

However when comparing the processing time of my creation to the PCL version the time differences are significant; My implementation is in the region of ~200ms per frame, whereas the implementation using PCL algorithms is running around ~10ms per frame.

For a real world lidar with a frame running at 10Hz then the faster processing of the pointcoud is essential to enable other programs to utilise the segmented data for self-driving functions.


---

## License

For License information please see the [LICENSE](./LICENSE) file for details

---

