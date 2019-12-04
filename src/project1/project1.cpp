/* \author Ahmed Fawzy */
// LiDAR Project 1

#include "../render/render.h"
#include "kdtree3D.h"
#include <unordered_set>
#include "../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../processPointClouds.cpp"

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


std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> Segmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	//For max iterations 
	for (int w = 0;w < maxIterations; w++)
	{
		std::unordered_set<int> inliers;
		// Randomly sample subset and fit line

		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		auto itr = inliers.begin();

      	float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		float z1 = cloud->points[*itr].z;

		itr++;

		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;
		float z2 = cloud->points[*itr].z;

		itr++;

		float x3 = cloud->points[*itr].x;
		float y3 = cloud->points[*itr].y;
		float z3 = cloud->points[*itr].z;	

		std::vector<float> v1; //Vector v1 travels from point1 to point2
		v1.push_back(x2-x1);
		v1.push_back(y2-y1);
		v1.push_back(z2-z1);

		std::vector<float> v2; //Vector v2 travels from point1 to point3
		v2.push_back(x3-x1);
		v2.push_back(y3-y1);
		v2.push_back(z3-z1);

		// Normal vector to the plane by taking cross product of v1 X v2
		// v1 X v2 = <i,j,k>
		std::vector<float> v1v2;
		v1v2.push_back((y2-y1)*(z3-z1)-(z2-z1)*(y3-y1));
		v1v2.push_back((z2-z1)*(x3-x1)-(x2-x1)*(z3-z1));
		v1v2.push_back((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1));

		float i = v1v2[0];
		float j = v1v2[1];
		float k = v1v2[2];
		
		float A = i;
		float B = j;
		float C = k;
		float D = (i*x1+j*y1+k*z1)*-1;
		
      	for (int ind = 0; ind < cloud->points.size();ind++)
        {
          	if (inliers.count(ind)>0)
			  continue;
			  
			pcl::PointXYZI point = cloud->points[ind];
          	float x = point.x;
      		float y = point.y;
			float z = point.z;

			// Measure distance between every point and fitted line
          	float distance = fabs(A*x+B*y+C*z+D)/sqrt(A*A+B*B+C*C);

		// If distance is smaller than threshold count it as inlier
          if (distance <= distanceTol)
          {
            inliers.insert(ind);
          }     
               
        }
      	if (inliers.size()>inliersResult.size())
        {
          // Return indicies of inliers from fitted line with most inliers
		  inliersResult = inliers;
        }
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>()); // Ground Plane
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>()); // Obstacles

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];

		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud(cloudOutliers, cloudInliers);
    
	return segmentedCloud;

}

void Proximity(int index, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>& singleClusterIDs, std::vector<bool>& isProcessed, KdTree3D* tree, float distanceTol)
{
	isProcessed[index] = true;

	singleClusterIDs.push_back(index);

	std::vector<int> nearbyIDs = tree->search3D(cloud->points[index], distanceTol);

	for (int id : nearbyIDs)
	{
		if(!isProcessed[id])
			Proximity(id, cloud, singleClusterIDs, isProcessed, tree, distanceTol);
	}
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, KdTree3D* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clustersIDs;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
	std::vector<bool> isProcessed(cloud->points.size(),false);
	
	int i = 0;
	while (i < cloud->points.size())
	{
		if(isProcessed[i])
		{
			i++;
			continue;
		}

		std::vector<int> singleClusterIDs;

		Proximity(i, cloud, singleClusterIDs, isProcessed, tree, distanceTol);

		if (singleClusterIDs.size() >= minSize && singleClusterIDs.size() <= maxSize)
		{
			clustersIDs.push_back(singleClusterIDs);
		}

		i++;
	}

	for(std::vector<int> singleClusterIDs : clustersIDs)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr singleCluster (new pcl::PointCloud<pcl::PointXYZI>());

  		for(int j : singleClusterIDs)
		  {
			singleCluster->points.push_back(cloud->points[j]);
		  }
		singleCluster->width = singleCluster->points.size();
      	singleCluster->height = 1;
      	singleCluster->is_dense = true;
      
      	cloudClusters.push_back(singleCluster);	
  	}
 
	return cloudClusters;
}

void cityBlockStream(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
	/* Filtering pointcloud; Downsampling and Region of Interest*/
	inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10,-5,-2,1), Eigen::Vector4f(30,8,1,1));
	
	/* Segmentation using RANSAC algorithm coded from scratch */
	std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = Segmentation(inputCloud, 25, 0.3);

	//renderPointCloud(viewer,segmentedCloud.first,"Obstacles",Color(1,0,0));
  	renderPointCloud(viewer,segmentedCloud.second,"Ground Plane",Color(0,1,0));

	pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud = segmentedCloud.first;

	/* 3D tree coded from scratch */
	KdTree3D* tree = new KdTree3D;

	for(int i = 0; i < obstCloud->points.size(); i++)
	{
		tree->insert3D(obstCloud->points[i],i);
	}

	/* Euclidean clustering coded from scratch */
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = Clustering(obstCloud, tree, 0.53, 10, 500);

	/* Rendering output clusters */
	int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1), Color(0,1,0), Color(1,1,1)};

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



int main (int argc, char** argv)
{
	
	std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
  
	/* Point processor instantiation in stack */
  	ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

	std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../../sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();

	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
	
	while (!viewer->wasStopped ())
    {
        // Clear viewer
  		viewer->removeAllPointClouds();
  		viewer->removeAllShapes();

  		/* Load pcd from the list*/
  		inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());

		/* Calling the core function for objects detection */
  		cityBlockStream(viewer, pointProcessorI, inputCloudI);

 	 	streamIterator++;
 		 if(streamIterator == stream.end())
    		streamIterator = stream.begin();

  		viewer->spinOnce ();
    } 	
}
;