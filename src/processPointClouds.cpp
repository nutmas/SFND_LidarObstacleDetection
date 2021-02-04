// PCL lib Functions for processing point clouds
#include <unordered_set>
#include "processPointClouds.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

// function to create ROI and voxelated cloud
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // object to downsample
    pcl::VoxelGrid<PointT> vg;

    // variable to hold filtered cloud
    typename pcl::PointCloud<PointT>::Ptr filteredPCL (new pcl::PointCloud<PointT> ());

    // pass cloud in for voxel processing
    vg.setInputCloud (cloud);
    // set the voxel cell size
    vg.setLeafSize (filterRes, filterRes, filterRes);
    //store results in the variable filterPCL
    vg.filter (*filteredPCL);

    // create a cloud region
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT> ());

    // create ROI for pcl to focus - create a crop box - true = extract removed indices (points inside crop box)
    pcl::CropBox<PointT> regionOfInterest(true);
    // set the max and min points to define ROI
    regionOfInterest.setMax(maxPoint);
    regionOfInterest.setMin(minPoint);
    regionOfInterest.setInputCloud(filteredPCL);
    // return the remainging ROI into the pcl - save into cloudRegion
    regionOfInterest.filter(*cloudRegion);

    // remove the roof area of the vehicle - do the opposite of the above
    std::vector<int> indices;

    // box to show roof area for removal
    float roofMinPointx, roofMinPointy, roofMinPointz;
    float roofMaxPointx, roofMaxPointy, roofMaxPointz;

    roofMinPointx = -1.8;  // rearwards
    roofMinPointy = -1.5; // right
    roofMinPointz = -1.0; // below lidar
    roofMaxPointx = 3.0; //forwards
    roofMaxPointy = 1.5; // left
    roofMaxPointz = 0.3;  // above lidar

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (roofMinPointx, roofMinPointy, roofMinPointz, 1));
    roof.setMax(Eigen::Vector4f (roofMaxPointx, roofMaxPointy, roofMaxPointz, 1));
    roof.setInputCloud(cloudRegion);
    // return the remaining ROI indices inside the roof boox area  into the vector
    roof.filter(indices);

    // add all the idices into the PointIndices
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices)
    {

        inliers->indices.push_back(point);
    }

    // create an extraction object - same as seperate point cloud segmentation
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true); // remove the points
    extract.filter(*cloudRegion); //remove the roof points

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process start
    auto startTime = std::chrono::steady_clock::now();

    // variable to hold best inliers
    std::unordered_set<int> inliersResult;
    // hold current results
    std::unordered_set<int> inliers; // ints so just index of points in cloud
    srand(time(NULL));

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;


    // perform for passed in maxIterations
    while(maxIterations--)
    {
        /* pick 3 random points from cloud to create plane */
        // populate set until 3 unique points
        while (inliers.size() < 3)
            inliers.insert(rand()%(cloud->points.size()));

        // point to start of inliers
        auto itr = inliers.begin();
        // get the points of cloud that are stored in the inliers position
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        /*
         * Equation of plane through 3 points
         * Ax + By + Cz + D = 0
         * vectors v1 * v2 = < i,j,k >
         */

        // Calculate A(i),B(j),C(k) values
        float i = ((y2-y1)*(z3-z1)) - ((z2-z1)*(y3-y1)); // A
        float j = ((z2-z1)*(x3-x1)) - ((x2-x1)*(z3-z1)); // B
        float k = ((x2-x1)*(y3-y1)) - ((y2-y1)*(x3-x1)); // C

        // Calculate the D value
        // D = -(ix1 + jy1 + kz1)
        float D = -((i*x1)+(j*y1)+(k*z1));

        float vec = sqrt(i*i+j*j+k*k);

        // iterate through all points in pcd to find distance
        for (int index = 0; index < cloud->points.size(); index++)
        {
            /* if point is not part of line */
            if(inliers.count(index)>0)
                continue; // escape loop to next loop

            // if point is not part of fitted line
            PointT point = cloud->points[index];

            // calculate distance point to plane
            float d = fabs(i*point.x+j*point.y+k*point.z+D)/vec;

            // add point if within threshold
            if(d <= distanceThreshold)
                inliers.insert(index);
        }

        //find best fit
        if (inliers.size()>inliersResult.size())
            inliersResult = inliers;

        // clear previous set
        inliers.clear();

    }

    // Time segmentation process end
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    // seperate the clouds into obstacles and road
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>()); // Road
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>()); // Obstacles

    for(int index = 0; index < cloud->points.size(); index++)
        {
            PointT point = cloud->points[index];
            if(inliersResult.count(index)>0)
                cloudInliers->points.push_back(point);
            else
                cloudOutliers->points.push_back(point);
        }

    // Return a std::pair of point clouds (obstacles, plane)
    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> (cloudOutliers, cloudInliers);
}



// recursive function; cluster and processed are passed by reference
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>> &points, std::vector<int>& cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol)
{


  if(processed[indice])
    return;
  // flag point as process
  processed[indice] = true;
  // push point into cluster
  cluster.push_back(indice);
  // call proximity function (point near node search)
  // search for nearest points
  std::vector<int> nearest = tree->search(points[indice], distanceTol);

  // loop through all nearest points
  for (int id : nearest)
  {
    // if point id hasn't been processed
    if(!processed[id])
      // recurse all points to get clusters
      clusterHelper(id, points, cluster, processed, tree, distanceTol);
  }
}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    // create list to hold clusters
    std::vector<std::vector<int>> clusters;
    // vector to track other points processed - initialise all to false
    std::vector<bool> processed(points.size(), false);

    int i = 0;
    // iterate through each point
    while(i < points.size())
    {
        // if point processed
        if(processed[i])
        {
          // increment and move to next point
          i++;
          continue;
        }
        // process false point
        // create cluster to hold points
        std::vector<int> cluster;
        // pass a point id, the points, cluster as reference, processed status, tree, tolerance
        clusterHelper(i, points, cluster, processed, tree, distanceTol);

        if(cluster.size() >= minSize && cluster.size() <= maxSize)
            // add cluster to clusters
            clusters.push_back(cluster);
        // increment to next point
        i++;
    }

    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 30, 1000);
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    // create a vector of point clouds to hold the returned clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;


    // vector to hold points data from cloud
    std::vector<std::vector<float>> points;
    // Creating the KdTree object for the search method of the extraction
    KdTree* tree = new KdTree;
    // pass entire source cloud into tree
    for (int i=0; i<cloud->points.size(); i++)
    {
        auto basePoint = cloud->points[i];
        // populate points vector
        points.push_back(std::vector<float> {basePoint.x, basePoint.y, basePoint.z});
        tree->insert(std::vector<float> {basePoint.x, basePoint.y, basePoint.z},i);

    }

    // return vector which holds clusters consisting of group indices - for each cluster cloud
    std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize);


    // itertate through the cluster indicies and create a new cloud for each group of clustered points
    int j = 0;
    //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)

    for(std::vector<int> item : cluster_indices)
    {
        // create a cloud
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        //loop through indices and copy points that are deemed in current cluster to new cluster cloud
        for(int index : item)
            // grab points member from cloud and piush it to a points memebr of cloud_cluster
            cloud_cluster->points.push_back(cloud->points[index]);

        clusters.push_back(cloud_cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

// used for single pcd frame loading into cloud variable
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
