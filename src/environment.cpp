/* \author Aaron Brown, Oleksandr Kashkevich */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <algorithm>
#include <limits>
#include <memory>
#include <vector>


enum Options {Streaming, Rendering};
enum StreamingOptions {StreamFrame=1, StreamStream};
enum RenderingOptions {RenderInputPC=1, RenderFilteredPC, RenderSegments, RenderClusters};


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


// SIMULATED PCD DATA RENDERING
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // RENDER OPTIONS
    bool renderScene = false;    // Change to true to visualize the scene
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Creating the LiDAR object
    std::shared_ptr<Lidar> lidar = std::make_shared<Lidar> (cars, 0.0);
    
    // Processing the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    
    // Visualizing the point cloud
    //renderRays(viewer, lidar->position, cloud);   // visualize the rays
    //renderPointCloud(viewer, cloud, "pc1", Color(0.5, 1, 0.5));   // visualize all the points together

    // Create point processor
    std::shared_ptr<ProcessPointClouds<pcl::PointXYZ>> processor = std::make_shared<ProcessPointClouds<pcl::PointXYZ>> ();
    
    // Segment the point cloud to a road point cloud and obstacles point cloud
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> roadAndObstacles = processor->SegmentPlane(cloud, 5, 0.3, true);
    
    // Visualizing the road
    renderPointCloud(viewer, roadAndObstacles.first, "roadCloud", Color(0,1,0));
    // Visualizing the obstacles
    //renderPointCloud(viewer, roadAndObstacles.second, "obstaclesCloud", Color(1,0,0));
    
    // Cluster the obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor->Clustering(roadAndObstacles.second, 1.0, 3, 30, true);
      
    // Render the clusters
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0.4, 0), Color(0, 0, 1)};
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        
        // Rendering the bounding boxes for the clouds
        // Box box = processor->BoundingBox(cluster);   // BBox without Z-axes rotation
        BoxQ box = processor->BoundingBoxQ(cluster);   // Minimum oriented BBox 
        
        if (clusterId % 3 == 0)
            renderBox(viewer, box, clusterId, Color(1, 0, 0));
        else if (clusterId % 3 == 1)
            renderBox(viewer, box, clusterId, Color(1, 1, 0));
        else
            renderBox(viewer, box, clusterId, Color(0, 0, 1));
        
        ++clusterId;
    }
}


void cityBlockStream(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud,
                     const int renderOption, bool drawBB, bool drawRoad, bool drawSelf=false)
{
    if (renderOption == RenderInputPC)
        renderPointCloud(viewer, inputCloud, "inputCloud");
  
    else {
        // Filtering the point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = processor->FilterCloud(inputCloud, .2, Eigen::Vector4f (-10, -6, -2, 1), Eigen::Vector4f (30, 6, 2, 1));
        
        if (renderOption == RenderFilteredPC) {
            renderPointCloud(viewer, filteredCloud, "outputCloud");
        
        } else {
            // Segment the point cloud
            std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> roadAndObstacles = processor->SegmentPlane(filteredCloud, 15, .25, true);
            
            if (drawRoad)
                renderPointCloud(viewer, roadAndObstacles.first, "roadCloud", Color(0,1,0));
            
            if (renderOption == RenderSegments)
                renderPointCloud(viewer, roadAndObstacles.second, "obstaclesCloud", Color(1,0,0));
            
            else {
                // Cluster the obstacles
                std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processor->Clustering(roadAndObstacles.second, .45, 10, 1000, true);
            
                // Render the clusters
                std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0.4, 0), Color(0, 0, 1)};
                int clusterId = 0;
                for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
                {
                    std::cout << "cluster size ";
                    processor->numPoints(cluster);
                    renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
                    
                    // Rendering the bounding boxes for the clouds
                    if (drawBB) {
                        Box box = processor->BoundingBox(cluster);   // BBox without Z-axes rotation
                        //BoxQ box = processor->BoundingBoxQ(cluster);   // Minimum oriented BBox 
                        
                        if (clusterId % 3 == 0)
                            renderBox(viewer, box, clusterId, Color(1, 0, 0));
                        else if (clusterId % 3 == 1)
                            renderBox(viewer, box, clusterId, Color(1, 1, 0));
                        else
                            renderBox(viewer, box, clusterId, Color(0, 0, 1));
                    }
                    
                    ++clusterId;
                }
            }
        }
    }
    
    // Optionally render the EgoCar
    if (drawSelf) {
        Car egoCar( Vect3(0, 0, -1.5), Vect3(4, 2, 2), Color(0, 0, 1), "egoCar");
        egoCar.render(viewer);
    }
}


// REAL PCD DATA RENDERING
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, const int renderOption, bool drawBB, bool drawRoad, bool drawSelf=false)
{
    // Creating Point Processor
    ProcessPointClouds<pcl::PointXYZI> *processor = new ProcessPointClouds<pcl::PointXYZI>();
    
    // Loading the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = processor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    cityBlockStream(viewer, processor, inputCloud, renderOption, drawBB, drawRoad);
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
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


void printStreamingOptions() {
    std::cout << "Please select an option (1-2):" << std::endl;
    std::cout << "1. Render single PCD file" << std::endl;
    std::cout << "2. Stream PCD files" << std::endl;
}


void printRenderingOptions() {
    std::cout << "Please select a rendering option (1-4):" << std::endl;
    std::cout << "1. Input point cloud" << std::endl;
    std::cout << "2. Filtered point cloud" << std::endl;
    std::cout << "3. Segmented road and obstacles clouds" << std::endl;
    std::cout << "4. Clustered obstacles" << std::endl;
}


template<typename Type>
void processInput(Type &option) {
    std::cin >> option;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}


int getOption(int optionType) {
    int option;
    std::vector<int> validOptions;
    void (*printFunc)() = NULL;
    
    if (optionType == Streaming) {
        validOptions = {1, 2};
        printFunc = &printStreamingOptions;
    } else {
        validOptions = {1, 2, 3, 4};
        printFunc = &printRenderingOptions;
    }
    
    printFunc();
    processInput(option);
    
    while (std::find(validOptions.begin(), validOptions.end(), option) == validOptions.end()) {
        std::cout << "\nInvalid value entered" << std::endl;
        printFunc();
        processInput(option);
    }
    
    return option;
}


bool getDrawingOption(std::string message) {
    char drawOption;
    std::cout << message;
    processInput(drawOption);
    
    while (std::tolower(drawOption) != 'y' && std::tolower(drawOption) != 'n') {
        std::cout << "\nInvalid value entered (enter 'y' or 'n') ";
        processInput(drawOption);
    }
    
    return (drawOption == 'y');
}


int main (int argc, char** argv)
{
    // Setting the options
    std::cout << "Welcome to PCD data visualization environment!" << std::endl;
    int streamingOption {getOption(Streaming)};
    int renderingOption {getOption(Rendering)};
    bool drawRoad, drawBB;
    
    if (renderingOption == RenderSegments || renderingOption == RenderClusters)
        drawRoad = getDrawingOption("Should the point cloud that belongs to the road be drawn? (y/n) ");
    
    if (renderingOption == RenderClusters)
        drawBB = getDrawingOption("Should the bounding boxes be drawn around the obstacles? (y/n) ");
    
    std::cout << "Starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);   // Using simulated data for rendering (one shot only)
    
    if (streamingOption == StreamFrame) {
        cityBlock(viewer, renderingOption, drawBB, drawRoad);
        
        while (!viewer->wasStopped ())
            viewer->spinOnce ();
    
    } else {
        ProcessPointClouds<pcl::PointXYZI> *processor = new ProcessPointClouds<pcl::PointXYZI>();
        std::vector<boost::filesystem::path> stream = processor->streamPcd("../src/sensors/data/pcd/data_1");
        auto streamIterator = stream.begin();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

        while (!viewer->wasStopped ()){   
            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            // Load pcd and run obstacle detection process
            inputCloud = processor->loadPcd((*streamIterator).string());
            cityBlockStream(viewer, processor, inputCloud, renderingOption, drawBB, drawRoad);

            streamIterator++;
            if(streamIterator == stream.end())
                streamIterator = stream.begin();
            
            viewer->spinOnce ();
        }
    }
}