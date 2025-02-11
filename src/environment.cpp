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

void cityBlockStream(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
   
    /* Filtering pointcloud; Downsampling and Region of Interest*/
    inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10,-5,-2,1), Eigen::Vector4f(30,8,1,1));

    /* Segmentation; RANSAC Algorithm*/
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI.SegmentPlane(inputCloud, 25, 0.3);
    //renderPointCloud(viewer, segmentedCloud.first, "Obstacle Cloud Red", Color(1,0,0));
    renderPointCloud(viewer, segmentedCloud.second, "Plane Cloud Green", Color(0,1,0));

    /* Clustring; KdTree and Eculidean*/
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentedCloud.first, 0.53, 10, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      std::cout << "cluster size ";
      pointProcessorI.numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      Box box = pointProcessorI.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
      ++clusterId;
    }

}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>(); //Heap

    /* Reading pointcloud file */
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    /* Filtering pointcloud; Downsampling and Region of Interest*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.1, Eigen::Vector4f(-20,-5,-5,1), Eigen::Vector4f(40,10,10,1));
    //renderPointCloud(viewer, filteredCloud, "filteredCloud");

    /* Segmentation; RANSAC Algorithm*/
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filteredCloud, 500, 0.2);
    //renderPointCloud(viewer, segmentedCloud.first, "Obstacle Cloud Red", Color(1,0,0));
    renderPointCloud(viewer, segmentedCloud.second, "Plane Cloud Green", Color(0,1,0));

    /* Clustring; KdTree and Eculidean*/
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentedCloud.first,0.2,3,750);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      std::cout << "cluster size ";
      pointProcessorI->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
      ++clusterId;
    }

}



void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, cloud, "Test", Color(0,0,255));


    // TODO:: Create point processor
    //ProcessPointClouds<pcl::PointXYZ> pointProcessor; //Stack
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); //Heap

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegCloud = pointProcessor->SegmentPlane(cloud, 100, 0.2);
    //renderPointCloud(viewer, SegCloud.first, "Obstacle Cloud Red", Color(1,0,0));
    //renderPointCloud(viewer, SegCloud.second, "Plane Cloud Green", Color(0,1,0));
  
  	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(SegCloud.first,1.0,3,30);
  
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
      std::cout << "cluster size ";
      pointProcessor->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      Box box = pointProcessor->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
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
  	//cityBlock(viewer);
  
  	ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
	std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
 

    while (!viewer->wasStopped ())
    {
        // Clear viewer
  		viewer->removeAllPointClouds();
  		viewer->removeAllShapes();

  		// Load pcd and run obstacle detection process
  		inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
  		cityBlockStream(viewer, pointProcessorI, inputCloudI);

 	 	streamIterator++;
 		 if(streamIterator == stream.end())
    		streamIterator = stream.begin();

  		viewer->spinOnce ();
    } 
}