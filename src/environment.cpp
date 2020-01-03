/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);


    // To get actual point cloud and its viz we need a lidar sensor type
    // TODO:: Create lidar sensor on heap memory pointer
    Lidar *lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = lidar->scan();
    // For rendering Rays
    //renderRays(viewer, lidar->position, point_cloud);
    //render Point clouds alone and not rays
    //renderPointCloud(viewer, point_cloud, "point_cloud");

    // PointProcessClouds type can be used for various point processing functions
    // TODO:: Create point processor on heap
    ProcessPointClouds<pcl::PointXYZ>* point_processor = new ProcessPointClouds<pcl::PointXYZ>();
    // on stack
    //ProcessPointClouds<pcl::PointXYZ> point_processor();

    // Segmentation of point cloud is the next step to understand the points
    // rendered above after scanning with lidar sensor
    std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr ,pcl::PointCloud<pcl::PointXYZ>::Ptr > segResult = point_processor->SegmentPlane_algo(point_cloud, 100, 0.2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud = segResult.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr road_cloud = segResult.second;
    // Render the two clouds
    //renderPointCloud(viewer, obstacle_cloud, "obstacle_cloud", Color(1,0,0));
    renderPointCloud(viewer, road_cloud, "road_cloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor->Clustering(obstacle_cloud, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
          //std::cout << "cluster size ";
          point_processor->numPoints(cluster);
          Box box = point_processor->BoundingBox(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
          //renderBox(viewer, box, clusterId, colors[clusterId]);
          ++clusterId;
    }
  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* point_processor, pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{

    //renderPointCloud(viewer, input_cloud, "input_cloud");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered = point_processor->FilterCloud(input_cloud, 0.2, Eigen::Vector4f(-20, -4, -4, 1), Eigen::Vector4f(20, 4, 4, 1));
    //renderPointCloud(viewer, cloud_filtered, "cloud_filtered");

    std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr ,pcl::PointCloud<pcl::PointXYZI>::Ptr > segResult = point_processor->SegmentPlane_algo(cloud_filtered, 1000, 0.2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud = segResult.first;
    pcl::PointCloud<pcl::PointXYZI>::Ptr road_cloud = segResult.second;
    // Render the two clouds
    //renderPointCloud(viewer, obstacle_cloud, "obstacle_cloud", Color(1,0,0));
    renderPointCloud(viewer, road_cloud, "road_cloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = point_processor->Clustering_algo(obstacle_cloud, 0.3, 10, 1000);

    int clusterId = 0;

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
          //std::cout << "cluster size ";
          //point_processor->numPoints(cluster);
          Box box = point_processor->BoundingBox(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
          renderBox(viewer, box, clusterId, colors[clusterId%3]);
          ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
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

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);     
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI= new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer->spinOnce ();
    }

}
