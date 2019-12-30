// PCL lib Functions for processing point clouds 

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


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{

    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Extraction of Segmented inlier points into seperate cloud
    // the outliers are the obstacles like cars on the road and the inliers are road points
    // declare two seperate point clouds
    // road point cloud which are inlier points taken based on inliers indices from point cloud
    typename pcl::PointCloud<PointT>::Ptr inlier_cloud{new pcl::PointCloud<PointT>};
    // obstacle poin clouds which are the rest of the outliers left in original point cloud
    typename pcl::PointCloud<PointT>::Ptr outlier_cloud{new pcl::PointCloud<PointT>};
    // Lets use inbuilt Extraction methods to seperate the point clouds
    // Create object on stack for extraction
    typename pcl::ExtractIndices<PointT> extract;
    // set the original input cloud
    extract.setInputCloud(cloud);
    // set the inlier point indices for extraction
    extract.setIndices(inliers);
    // to filter by subtracting the inliers from cloud and get outliers set true
    extract.setNegative(true);
    // call filter for extraction of outlier
    extract.filter(*outlier_cloud);
    // to filter inlier points based on inlier indices
    extract.setNegative(false);
    // call filter for extraction of inlier
    extract.filter(*inlier_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(outlier_cloud, inlier_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // This is the output point indices for the input point cloud after segmentation
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.

    //Segmentation using inbuild pcl functions
    // This is the object on stack which enables inbuilt segmentation
    typename pcl::SACSegmentation<PointT> seg;
    //used for rendering the plane on pcl viewer
    seg.setOptimizeCoefficients(true);
    // segmentation will happen with plane model. So lets set that
    seg.setModelType(pcl::SACMODEL_PLANE);
    // the best plane is found using the RANSAC optimisation algorthm
    seg.setMethodType(pcl::SAC_RANSAC);
    // This sets the number of iterations. For each iteration a model
    // is chosen and its max inliers count are found for each iteration
    seg.setMaxIterations(maxIterations);
    // the distance threshold for planar ransac algorithm
    seg.setDistanceThreshold(distanceThreshold);
    // Set input point cloud data
    seg.setInputCloud(cloud);
    // Actual segmentation
    // ModelCoefficients
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // call inbuilt segmentation function
    seg.segment(*inliers, *coefficients);
    // sanity check for results
    if(inliers->indices.size() == 0)
    {
        std::cout << "cant estimate planar model from given dataset " << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // After Segmentation we get inlier indices, thus we need to separate inliers and outliers from cloud
    // based on these indices
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_algo(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // This is the output point indices for the input point cloud after segmentation
    //pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.

    //Segmentation
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

    // For max iterations
    while(maxIterations--)
    {

    // Randomly sample subset and fit line
        std::unordered_set<int> inliers;
        // Add two unique random point indices for line model
        while(inliers.size() < 3)
        {
            inliers.insert(rand()%cloud->points.size());
        }
        // point values from the unique random indices
        float x1,y1,z1,x2,y2,z2,x3,y3,z3;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        *itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        *itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // Line equation coefficints from the points
        float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        float d = -(a*x3 + b*y3 + c*z3);

        // Iterate through every point in the cloud to get distance
        for (int index = 0; index < cloud->points.size(); ++index)
        {
            // If the point is in the line model skip
            if(inliers.count(index) > 0)
                continue;

            // Each Point of cloud
            float x0 = cloud->points[index].x;
            float y0 = cloud->points[index].y;
            float z0 = cloud->points[index].z;

            // Measure distance between every point and fitted line
            // If distance is smaller than threshold count it as inlier

            // Measure distance
            float dist = fabs(a*x0 + b*y0 + c*z0 + d) / sqrt(a*a + b*b + c*c);

            // check against threshold
            if (dist < distanceThreshold)
            {
                inliers.insert(index);
            }
        }

        if(inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }
    // sanity check for results
    if(inliersResult.size() == 0)
    {
        std::cout << "cant estimate planar model from given dataset " << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new typename pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new typename pcl::PointCloud<PointT>());


    // After Segmentation we get inlier indices, thus we need to separate inliers and outliers from cloud
    // based on these indices
    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Use built in pcl functions for clustering
    // KDtree object for clustering the points
    typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
    // set Input cloud which arranges the cloud points in binary tree form
    tree->setInputCloud(cloud);

    // To Extract the clusters from binary tree we need Euclidean cluster extraction object
    typename pcl::EuclideanClusterExtraction<PointT> ec;

    // vector of clusters which are collection of indices referenced in the cloud input
    std::vector<pcl::PointIndices> cluster_indices;

    // parameters for the extraction of clusters from the Kdtree of cloud points
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    // Do extraction
    ec.extract(cluster_indices);

    for(pcl::PointIndices cluster_idx: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster {new typename pcl::PointCloud<PointT>};
        for(int idx: cluster_idx.indices)
        {
            cluster->points.push_back(cloud->points[idx]);
        }
        cluster->width = cluster_idx.indices.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
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
