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
    
    // TODO:: Create lidar sensor 

    Lidar *lidar = new Lidar(cars, 0); 
    pcl::PointCloud<pcl::PointXYZ> :: Ptr input_cloud = lidar -> scan();

    // renderRays(viewer, lidar -> position, input_cloud); 
    // renderPointCloud(viewer, input_cloud, "input lidar cloud"); 
    // TODO:: Create point processor  
    ProcessPointClouds<pcl::PointXYZ> *point_processer = new ProcessPointClouds<pcl::PointXYZ>; 
    std::pair <pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedCloud = (*point_processer).SegmentPlane(input_cloud, 100, 0.2);
    //First is road and second is obstacle
    renderPointCloud(viewer, segmentedCloud.first, "road", Color(0, 1, 0)); 
    // renderPointCloud(viewer, segmentedCloud.second, "obstacles", Color(1, 0, 0)); 

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
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}