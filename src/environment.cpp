#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <typeinfo>

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
    
    // TODO:: Create lidar sensor 

    Lidar *lidar = new Lidar(cars, 0); 
    pcl::PointCloud<pcl::PointXYZ> :: Ptr input_cloud = lidar -> scan();

    // renderRays(viewer, lidar -> position, input_cloud); 
    ProcessPointClouds<pcl::PointXYZ> *point_processer = new ProcessPointClouds<pcl::PointXYZ>; 
    std::pair <pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedCloud = (*point_processer).SegmentPlane(input_cloud, 100, 0.2);
    renderPointCloud(viewer, segmentedCloud.first, "road", Color(0, 1, 0)); 

    // Clustering example with pcl library and cloud output.
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster = point_processer -> Clustering(segmentedCloud.second, 2.5, 3, 30); 

    int cluster_id = 0; 
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)}; 
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr single_cluster : cloud_cluster){
        std::cout << "cluster number " << cluster_id << " : "; 
        point_processer -> numPoints(single_cluster); 

        renderPointCloud(viewer, single_cluster, "Obst Cloud" + std::to_string(cluster_id), colors[cluster_id % cloud_cluster.size()]); 
        Box box = point_processer->BoundingBox(single_cluster);
        renderBox(viewer,box,cluster_id);
        cluster_id++; 
    }
  
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* cityBlock_point_processor, 
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_Cloud){

    // ProcessPointClouds<pcl::PointXYZI>* cityBlock_point_processor = new ProcessPointClouds<pcl::PointXYZI>(); 
    // pcl::PointCloud<pcl::PointXYZI>::Ptr  input_Cloud = cityBlock_point_processor -> loadPcd("/Users/vishal.jain/Documents/LidarProject/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd"); 

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> filtered_cloud; 
    filtered_cloud = cityBlock_point_processor -> FilterCloud(input_Cloud, 0.3f, Eigen::Vector4f (-10, -6, -3, 1), Eigen::Vector4f (30, 7, 3, 1));
    
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_plane = cityBlock_point_processor -> RansacModel(
        filtered_cloud.first, 
        10, 0.5
    ); 

    // Changed Clustering in this function to euclidian clustering
    KdTree* kdtree (new KdTree()); 
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = cityBlock_point_processor -> euclideanCluster(segmented_plane.second -> points, kdtree, 0.8f);
    // Convert PointXYX to floating point 
    std::vector<std::vector<float>> points; 
    for (pcl::PointCloud<pcl::PointXYZI>::iterator it1 = segmented_plane.second -> begin(); it1 != segmented_plane.second -> end(); ++it1){
        std::vector<float> new_point; 
        new_point.push_back(it1 -> x); 
        new_point.push_back(it1 -> y); 
        new_point.push_back(it1 -> z); 

        points.push_back(new_point); 
    }

    // Passing in points in some order and will be returned a vector of grouped indices
    std::vector<std::vector<int>> clusters_indices = cityBlock_point_processor -> euclideanCluster(points, kdtree, 0.4f);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters; 
    for(std::vector<int> cluster : clusters_indices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        for(int indice: cluster){
            clusterCloud->points.push_back(pcl::PointXYZI(points[indice][0],points[indice][1],points[indice][2]));
        }
        clusters.push_back(clusterCloud); 
    }


    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(1, 1, 0), Color(0.5, 0.5, 0.5)}; 
    int cluster_id  = 0; 
    for (auto single_cluster : clusters){
        renderPointCloud(viewer, single_cluster, std::to_string(cluster_id), colors[cluster_id % colors.size()]);
        Box single_box = cityBlock_point_processor -> BoundingBox(single_cluster); 
        renderBox(viewer, single_box, cluster_id, colors[cluster_id % colors.size()], 0.75); 

        cluster_id++; 

    }


    renderPointCloud(viewer, segmented_plane.first, "road", Color(0, 1, 0));

    Box ego_box = cityBlock_point_processor -> BoundingBox(filtered_cloud.second); 
    renderBox(viewer, ego_box, -1, Color(1, 0, 1)); 

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
    CameraAngle setAngle = TopDown;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* point_processor = new ProcessPointClouds<pcl::PointXYZI>; 
    std::vector<boost::filesystem::path> pcd_file_paths = point_processor -> streamPcd("/Users/vishal.jain/Documents/LidarProject/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2"); 
    auto stream_iterator(pcd_file_paths.begin()); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_CloudI; 


    while (!viewer->wasStopped ())
    {
        viewer -> removeAllPointClouds(); 
        viewer -> removeAllShapes(); 

        input_CloudI = point_processor -> loadPcd((*stream_iterator).string()); 
        cityBlock(viewer, point_processor, input_CloudI); 

        stream_iterator++; 
        if (stream_iterator == pcd_file_paths.end()){
            stream_iterator = pcd_file_paths.begin(); 
        }

        viewer->spinOnce ();
    } 
}