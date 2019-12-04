/* \author Ahmed Fawzy */

#include "../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI pnt, int setId)
	:	point(pnt), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree3D
{
	Node* root;

	KdTree3D()
	: root(NULL)
	{}

	void insertHelper3D(Node** node, uint depth, pcl::PointXYZI point, int id)
	{
		if(*node==NULL)
			*node = new Node(point,id);
		else
		{
			uint cd = depth % 3; //Current Dimension, Instead of 2 as we're working with 3D data

			switch (cd)
			{
				case 0:
					if(point.x < ((*node)->point.x))
						insertHelper3D(&((*node)->left), depth+1, point, id);
					else
						insertHelper3D(&((*node)->right), depth+1, point, id);
					break;

				case 1:
					if(point.y < ((*node)->point.y))
						insertHelper3D(&((*node)->left), depth+1, point, id);
					else
						insertHelper3D(&((*node)->right), depth+1, point, id);
					break;

				case 2:
					if(point.z < ((*node)->point.z))
						insertHelper3D(&((*node)->left), depth+1, point, id);
					else
						insertHelper3D(&((*node)->right), depth+1, point, id);
					break;
				
				default:
					break;
			}	
		}	
	}
	
	void insert3D(pcl::PointXYZI point, int id)
	{
		insertHelper3D(&root, 0, point, id);
	}


	void searchHelper3D(pcl::PointXYZI target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{
			/* The point lies within the cube around target point? */
			if((node->point.x>=(target.x-distanceTol) &&  node->point.x<=(target.x+distanceTol)) && (node->point.y>=(target.y-distanceTol) &&  node->point.y<=(target.y+distanceTol)) && (node->point.z>=(target.z-distanceTol) &&  node->point.z<=(target.z+distanceTol)))
			{
				/* Distance between 2 points in 3D */
				float distance = sqrt((node->point.x-target.x) * (node->point.x-target.x) + (node->point.y-target.y) * (node->point.y-target.y) + (node->point.z-target.z) * (node->point.z-target.z));
				if(distance <= distanceTol)
					ids.push_back(node->id);			
			}

			uint cd = depth % 3;

			switch (cd)
			{
				case 0:
					if((target.x-distanceTol) < node->point.x)
						searchHelper3D(target, node->left, depth+1, distanceTol, ids);

					if((target.x+distanceTol) > node->point.x)
						searchHelper3D(target, node->right, depth+1, distanceTol, ids);
					break;

				case 1:
					if((target.y-distanceTol) < node->point.y)
						searchHelper3D(target, node->left, depth+1, distanceTol, ids);

					if((target.y+distanceTol) > node->point.y)
						searchHelper3D(target, node->right, depth+1, distanceTol, ids);
					break;

				case 2:
					if((target.z-distanceTol) < node->point.z)
						searchHelper3D(target, node->left, depth+1, distanceTol, ids);

					if((target.z+distanceTol) > node->point.z)
						searchHelper3D(target, node->right, depth+1, distanceTol, ids);
					break;
				
				default:
					break;
			}

		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search3D(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper3D(target, root, 0, distanceTol, ids);
		return ids;
	}  		

};




