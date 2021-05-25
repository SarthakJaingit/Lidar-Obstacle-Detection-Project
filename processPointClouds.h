// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	void insert_helper(Node** node, uint depth, int id, std::vector <float> point){

		// Base Case

		if (*node == NULL){
			*node = new Node(point, id); 
		}
		else {
			// Recursive Functionality
			if (point[depth % 3]  < ((*node) -> point[depth % 3])){
				insert_helper(&((*node) -> left), ++depth, id, point); 
			}
			else {
				insert_helper(&((*node) -> right), ++depth, id, point); 
		}

		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(&root, 0, id, point); 
		}


	void search_helper(std::vector<float> target, Node* starting_node, float distanceTol, 
	std::vector<int>& ids, uint depth = 0)
	{

		if (starting_node != NULL){
			// Less expensive distance checking
			if (starting_node -> point[0] >= target[0] - distanceTol
			&& starting_node -> point[0] <= target[0] + distanceTol
			&& starting_node -> point[1] >= target[1] - distanceTol
			&& starting_node -> point[1] <= target[1] + distanceTol
			&& starting_node -> point[2] >= target[2] - distanceTol
			&& starting_node -> point[2] <= target[2] + distanceTol
			){
				 //More expensive distance check
				 float dis = sqrtf(pow(target[0] - starting_node -> point[0], 2) + pow(target[1] - starting_node -> point[1], 2) + \
				 pow(target[2] - starting_node -> point[2], 2));  
				 
				 if (dis <= distanceTol){
					 ids.push_back(starting_node -> id); 
					 }
			}


			if (target[depth % 3] - distanceTol < starting_node -> point[depth % 3])
			{
				// if lowest tolerance < point -> look into low tolerant area
				search_helper(target, starting_node -> left, distanceTol, ids, ++depth); 
			}
			if (distanceTol + target[depth % 3] > starting_node -> point[depth % 3])
			{
				// if highest tolerance > point -> look in high tolerant area
				search_helper(target, starting_node -> right, distanceTol, ids, ++depth); 
			}

	}

	}
	
		
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		
		std::vector<int> ids; 
		search_helper(target, root, distanceTol, ids); 
		return ids;
	}
	

};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacModel(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold); 

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    void Proxim(const std::vector<std::vector<float>>& points, int point_id, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol); 

    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol); 

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};


#endif /* PROCESSPOINTCLOUDS_H_ */