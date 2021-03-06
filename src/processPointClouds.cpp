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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Use voxels to lower the amount of data points

    pcl::VoxelGrid<PointT> sor; 
    sor.setInputCloud(cloud); 
    sor.setLeafSize(filterRes, filterRes, filterRes); 
    sor.filter(*cloud); 

    // Crop the detected points that are far away from the car. 

    pcl::CropBox<PointT> crop_field(true); 
    crop_field.setMax(maxPoint); 
    crop_field.setMin(minPoint);
    crop_field.setInputCloud(cloud); 
    crop_field.filter(*cloud); 

    // Use crop box to find indices that could be ego car roof and remove them
    std::vector<int> indices;
    typename pcl::PointCloud<PointT>::Ptr ego_cluster (new pcl::PointCloud<PointT>()); 

    pcl::CropBox<PointT> crop_roof(true); 
    crop_roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1)); 
    crop_roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1)); 
    crop_roof.setInputCloud(cloud); 
    crop_roof.filter(indices); 
    
    crop_roof.filter(*ego_cluster); 

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); 
    for (int index : indices){
        inliers -> indices.push_back(index); 
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract; 
    extract.setInputCloud(cloud); 
    extract.setIndices(inliers); 
    extract.setNegative(true); 
    extract.filter(*cloud); 

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> filtered_clouds(cloud, ego_cluster); 
    return filtered_clouds; 

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients); 
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
    
    seg.setOptimizeCoefficients(true); 
    
    seg.setModelType(pcl::SACMODEL_PLANE); 
    seg.setMethodType(pcl::SAC_RANSAC); 
    seg.setMaxIterations(maxIterations); 
    seg.setDistanceThreshold(distanceThreshold); 

    seg.setInputCloud(cloud); 
    seg.segment(*inliers, *coefficient); 
    if (inliers -> indices.size() == 0){
        std::cout << "Could not estimate for given dataset" << std::endl; 
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>); 
    typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>); 

    pcl::ExtractIndices<PointT> extract; 
    
    extract.setInputCloud(cloud); 
    extract.setIndices(inliers); 
    extract.setNegative(false); 
    extract.filter(*road); 

    extract.setNegative(true); 
    extract.filter(*obstacles); 

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(road, obstacles);
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>); 
    typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>); 

    for (int index = 0; index < cloud -> points.size(); ++index){
        auto current_XYZ = cloud -> points[index]; 
        //Look in big unordered set, inliers, for the integer index and count it. 
        if (inliers.count(index)){
            road -> points.push_back(current_XYZ); 
        } 
        else{
            obstacles -> points.push_back(current_XYZ); 
        }

    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(road, obstacles);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacModel(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    std::unordered_set<int> inliersResult;
	for (int i = 0; i < maxIterations; ++i){

		std::unordered_set<int> inliers; 
		while (inliers.size() <= 3){
			// {index 1, index 2, index 3}
			inliers.insert((rand() % cloud -> points.size())); 
		}

		std::unordered_set<int>::iterator it = inliers.begin(); 
		float x1 = cloud -> points[*it].x; 
		float y1 = cloud -> points[*it].y; 
		float z1 = cloud -> points[*it].z; 

		++it; 

		float x2 = cloud -> points[*it].x; 
		float y2 = cloud -> points[*it].y; 
		float z2 = cloud -> points[*it].z; 

		++it; 

		float x3 = cloud -> points[*it].x; 
		float y3 = cloud -> points[*it].y; 
		float z3 = cloud -> points[*it].z;


		float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1); 
		float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1); 
		float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1); 
		float d = -(a * x1 + b * y1 + c * z1); 

		for (int i = 0; i < cloud -> points.size(); ++i){

			if (inliers.count(i)){
				continue; 
			}

			float x4 = cloud ->points[i].x; 
			float y4 = cloud ->points[i].y; 
			float z4 = cloud ->points[i].z; 

			float distanceh = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(pow(a,2) + pow(b,2) + pow(c,2)); 

			if (distanceh < distanceThreshold){
				inliers.insert(i); 
			}

		if (inliers.size() > inliersResult.size()){
			inliersResult = inliers; 

		} 

		}

	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>); 
    tree -> setInputCloud(cloud); 

    std::vector<pcl::PointIndices> cluster_indices; 
    pcl::EuclideanClusterExtraction<PointT> euclidean_extracter; 

    euclidean_extracter.setClusterTolerance(clusterTolerance); 
    euclidean_extracter.setMinClusterSize(minSize); 
    euclidean_extracter.setMaxClusterSize(maxSize); 
    euclidean_extracter.setSearchMethod(tree); 
    euclidean_extracter.setInputCloud(cloud); 
    euclidean_extracter.extract(cluster_indices); 

    for (pcl::PointIndices single_cluster_set : cluster_indices){
        
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>); 
        for (int index : single_cluster_set.indices){
            cloud_cluster -> points.push_back(cloud -> points[index]); 

        }

        cloud_cluster -> width = cloud_cluster -> points.size(); 
        cloud_cluster -> height = 1; 
        cloud_cluster -> is_dense = true; 

        clusters.push_back(cloud_cluster); 
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proxim(const std::vector<std::vector<float>>& points, int point_id, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol){

	processed[point_id] = true; 
	cluster.push_back(point_id); 

	std::vector<int> close_by = tree -> search(points[point_id], distanceTol); 
	for (int id : close_by){
		if (!processed[id]){
			Proxim(points, id, cluster, processed, tree, distanceTol); 
		}
		
	}

}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false); 

	for (int id = 0; id < points.size(); ++id){
		if (!processed[id]){
			std::vector<int> cluster;
			Proxim(points, id, cluster, processed, tree, distanceTol); 
			clusters.push_back(cluster); 
		}

	}
 
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


// Clustering cars using KD Tree and Euclidian Clustering

// template<typename PointT>
// std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
// {

//     // Time clustering process
//     auto startTime = std::chrono::steady_clock::now();

//     std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

//     // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

//     typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>); 
//     tree -> setInputCloud(cloud); 

//     std::vector<pcl::PointIndices> cluster_indices; 
//     pcl::EuclideanClusterExtraction<PointT> euclidean_extracter; 

//     euclidean_extracter.setClusterTolerance(clusterTolerance); 
//     euclidean_extracter.setMinClusterSize(minSize); 
//     euclidean_extracter.setMaxClusterSize(maxSize); 
//     euclidean_extracter.setSearchMethod(tree); 
//     euclidean_extracter.setInputCloud(cloud); 
//     euclidean_extracter.extract(cluster_indices); 

//     for (pcl::PointIndices single_cluster_set : cluster_indices){
        
//         typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>); 
//         for (int index : single_cluster_set.indices){
//             cloud_cluster -> points.push_back(cloud -> points[index]); 

//         }

//         cloud_cluster -> width = cloud_cluster -> points.size(); 
//         cloud_cluster -> height = 1; 
//         cloud_cluster -> is_dense = true; 

//         clusters.push_back(cloud_cluster); 
//     }

//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

//     return clusters;
// }