#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

bool render_obst = false;
bool render_plane = false;
bool render_box = true;
bool render_clusters = false;
bool render_original_data = false;
bool render_filter = true;


void realTime(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // render the poincloud passed in
    if(render_original_data)
        renderPointCloud(viewer,inputCloud,"inputCloud", Color(1,1,1));

    // Hyperparameters
    float voxelSize = 0.2;              // voxel size
    float roiMinPointx = -8.0;         // ROI rearwards
    float roiMinPointy = -6.0;          // ROI to right
    float roiMinPointz = -2.0;          // ROI below lidar
    float roiMaxPointx = 30.0;          // ROI forwards
    float roiMaxPointy = 6.0;           // ROI to left
    float roiMaxPointz = 0.6;           // ROI above lidar

    int segMaxIterations = 100;          // number of cycles for segment
    float segDistanceThreshold = 0.15;   // distance between point data - offset from floor

    float clusterPointDistance = 0.5;
    int minClusterSize = 15;
    int maxClusterSize = 600;

    // modify cloud to only contain region of interest, and reduce to voxels
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, voxelSize , Eigen::Vector4f (roiMinPointx, roiMinPointy, roiMinPointz, 1), Eigen::Vector4f ( roiMaxPointx, roiMaxPointy, roiMaxPointz, 1));

    if(render_filter)
        renderPointCloud(viewer,filterCloud,"filterCloud", Color(1,1,1));

    // seperate point cloud into obstacles(first) and road(second)
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, segMaxIterations, segDistanceThreshold);

    if(render_obst)
        renderPointCloud(viewer,segmentCloud.first,"obstacles",Color(1,1,1));
    if (render_plane)
        renderPointCloud(viewer,segmentCloud.second,"road",Color(1,1,1));

    // cluster the objects cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterPointDistance, minClusterSize, maxClusterSize);

    // Box roiBox;

    // roiBox.x_min = -1.8;  // rearwards
    // roiBox.y_min = -1.5; // right
    // roiBox.z_min = -1.0; // below lidar
    // roiBox.x_max = 3.0; //forwards
    // roiBox.y_max = 1.5; // left
    // roiBox.z_max = 0.3;  // above lidar

    //renderBox(viewer, roiBox, 89, Color(0.5,1,0.2));


    // colour each obstacle cluster a different colour
    int clusterId = 0;
    //create a vector to hold the RGB values
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(0,1,1), Color(1,0,1)};

    // iterate through clusters of points in pointcloud cluster
    // a cluster ID should have been appended to the obsCloud string
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

        }

        if(render_box)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            // call function in render.cpp
            renderBox(viewer, box, clusterId, Color(0.5, 0.5, 0)); // yellow colour for box
        }
        // move to next cluster
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
// viewer passed in as reference meaning any changes made in here will persist once function is complete
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    // create a viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // options: XY (45deg), TopDown, Side, FPS (First person Sense - Drivers Seat)
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // create instance of class
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    // instantiate a PCD for input data
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    /* for single instance use case */
    // populate inputCloudI with data - single frame - returns a cloud
    //inputCloudI = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // pass the PCD viewer into function
    //realTime(viewer,pointProcessorI, inputCloudI);


    // realTime application
    // create a pointProcessor instance created on the heap (slower than stack, but larger space)
    // use streamPcd from pointProcessor to read the folder with the PCD stream data
    // returns a chronologically ordered vector of the file names in the directory
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    // // iterate through vector stream
    auto streamIterator = stream.begin();

    while (!viewer->wasStopped ())
    {
        /**
         * Add viewer run cycle for stream pcd
         */

        // clear the viewer of previous
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //Load pcd and run obstacle detection process - file path is dereference value
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        // function to process cloud
        realTime(viewer, pointProcessorI, inputCloudI);

        //increment iterator
        streamIterator++;

        //function to reset iterator, to repeat loop again
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        // run next viewer loop - used for both stream and single pcd frame
        viewer->spinOnce ();
    }
}
