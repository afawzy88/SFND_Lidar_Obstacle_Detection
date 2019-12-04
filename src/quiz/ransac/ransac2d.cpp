/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
  
  	// TODO: Fill in this function
  	float A, B, C, distance;
  	float x1,y1,x2,y2,x3,y3;

	//For max iterations 
	for (int i = 0;i < maxIterations; i++)
	{
		std::unordered_set<int> inliers;
		// Randomly sample subset and fit line
      	pcl::PointXYZ randPoint1 = cloud->points[rand()%cloud->points.size()];
      	pcl::PointXYZ randPoint2 = cloud->points[rand()%cloud->points.size()];
      	x1 = randPoint1.x;
      	y1 = randPoint1.y;
      	x2 = randPoint2.x;
      	y2 = randPoint2.y;
      	A = y1 - y2;
      	B = x2 - x1;
      	C = x1*y2 - x2*y1;
      	for (int j = 0; j < cloud->points.size();j++)
        {
          	pcl::PointXYZ point = cloud->points[j];
          	x3 = point.x;
      		y3 = point.y;
			// Measure distance between every point and fitted line
          	distance = fabs(A*x3+B*y3+C)/sqrt(A*A+B*B);
		// If distance is smaller than threshold count it as inlier
          if (distance <= distanceTol)
          {
            inliers.insert(j);
          }     
               
        }
      	if (inliers.size()>inliersResult.size())
        {
          // Return indicies of inliers from fitted line with most inliers
		  inliersResult = inliers;
        }
	}

	return inliersResult;

}



std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

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
			  
			pcl::PointXYZ point = cloud->points[ind];
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

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// Create 3D data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D = CreateData3D();

	ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); //Heap

    /* Reading pointcloud file */
    pcl::PointCloud<pcl::PointXYZ>::Ptr readCloud = pointProcessor->loadPcd("../../../../src/sensors/data/pcd/data_1/0000000000.pcd");

	/* Filtering pointcloud; Downsampling and Region of Interest*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D = pointProcessor->FilterCloud(readCloud, 0.1, Eigen::Vector4f(-20,-5,-5,1), Eigen::Vector4f(40,10,10,1));
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud3D, 300, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud3D->points.size(); index++)
	{
		pcl::PointXYZ point = cloud3D->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud3D,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
