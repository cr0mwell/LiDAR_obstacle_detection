/* \author Aaron Brown, Oleksandr Kashkevich */
// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include <string>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::cout << cloud->points.size() << std::endl;
}


/*#################
 * FILTERING
 *#################*/

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint,
                                                                              Eigen::Vector4f maxPoint) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid point reduction
    typename pcl::PointCloud<PointT>::Ptr outputCloud {new pcl::PointCloud<PointT> ()};
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr finalCloud(new pcl::PointCloud<PointT>);
    
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloudFiltered);
    
    // Region based filtering
    pcl::CropBox<PointT> cb;
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.setInputCloud(cloudFiltered);
    cb.filter(*outputCloud);
    
    // Removing points of the car roof
    // Collect roof inliers indices
    std::vector<int> inliers;
    pcl::PointIndices::Ptr roofInliers(new pcl::PointIndices);
    pcl::CropBox<PointT> cbr;
    cbr.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    cbr.setMax(Eigen::Vector4f(2.6, 1.7, 0, 1));
    cbr.setInputCloud(outputCloud);
    cbr.filter(inliers);
    
    for (const auto &i: inliers)
        roofInliers->indices.push_back(i);
    
    // Extract the outliers (points behind the roof) into the final point cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(outputCloud);
    extract.setIndices(roofInliers);
    extract.setNegative(true);
    extract.filter(*finalCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return finalCloud;
}


/*#################
 * SEGMENTATION
 *#################*/

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    size_t p_size = cloud->points.size();
    srand(time(NULL));
    
    // RANSAC implementation
    while (maxIterations > 0) {
        std::unordered_set<int> inliers;
        
        while (inliers.size() < 3)
            inliers.insert(rand() % p_size);
        
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
        
        float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        float b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
        float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
        float d = -(a*x1 + b*y1 + c*z1);

        for (size_t i {}; i < p_size; i++){
            // We shouldn't determine the distance to the points that are forming the plane
            if (inliers.count(i) > 0)
                continue;
            
            float x4 = cloud->points[i].x;
            float y4 = cloud->points[i].y;
            float z4 = cloud->points[i].z;
            
            float dist = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);
            
            if (dist <= distanceTol) 
                inliers.insert(i);
            
            if (inliers.size() > inliersResult.size())
                inliersResult = inliers;
        }
        maxIterations--;
    }
    
    return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                                                                                                                   typename pcl::PointCloud<PointT>::Ptr cloud) {
    typename pcl::PointCloud<PointT>::Ptr obstacles {new pcl::PointCloud<PointT> ()};
    typename pcl::PointCloud<PointT>::Ptr road {new pcl::PointCloud<PointT> ()};
    
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    
    // Extract the inliers (road points)
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road);
    
    // Extract the outliers (obstacles points)
    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(road, obstacles);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                                                                 int maxIterations,
                                                                                                                                 float distanceThreshold,
                                                                                                                                 bool useCustomAlgorithm) {
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    if (useCustomAlgorithm) {
        std::cout << "Using custom segmentation algorithm" << std::endl;
        std::unordered_set<int> inliers_set = Ransac3D<PointT>(cloud, maxIterations, distanceThreshold);
        inliers->indices.insert(inliers->indices.end(), inliers_set.begin(), inliers_set.end());
        
    } else {
        pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
        
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients (true);
        
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (maxIterations);
        seg.setDistanceThreshold (distanceThreshold);
        
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
    }
    
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    // Separating the obstacles from the road plane
    // Return a pair <road, obstacles>
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


/*#################
 * CLUSTERING
 *#################*/
 
// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
    PointT point;
    int id;
    Node* left;
    Node* right;

    Node(const PointT &arr, const int &setId)
    :   point(arr), id(setId), left(NULL), right(NULL)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
    
    bool isLargerThan(const PointT &p2, const unsigned &sublevel) {
        if (sublevel % 3 == 0)
            return point.x > p2.x;
        else if (sublevel % 3 == 1)
            return point.y > p2.y;
        else
            return point.z > p2.z;
    }
    
    bool isWithinDistanceTo(const PointT &p2, const float distanceTol) {
        return point.x >= (p2.x - distanceTol) &&  point.x <= (p2.x + distanceTol) &&
               point.y >= (p2.y - distanceTol) &&  point.y <= (p2.y + distanceTol) &&
               point.z >= (p2.z - distanceTol) &&  point.z <= (p2.z + distanceTol);
    }
    
    float getDistanceTo(const PointT &p2) {
        float xLen = point.x - p2.x;
        float yLen = point.y - p2.y;
        float zLen = point.z - p2.z;
        return sqrt(xLen*xLen + yLen*yLen + zLen*zLen);
    }
};


template<typename PointT>
struct KdTree
{
    Node<PointT>* root;

    KdTree() : root(NULL) {}

    ~KdTree() {
        delete root;
    }
    
    void insert(Node<PointT>*& node, unsigned sublevel, const PointT &point, const int &id) {
        if(node == NULL)
            node = new Node<PointT>(point, id);
        else if(node->isLargerThan(point, sublevel))
            insert(node->left, sublevel+1, point, id);
        else
            insert(node->right, sublevel+1, point, id);
    }

    void helperSearch(pcl::PointIndices &ids, uint sublevel, Node<PointT>*& node, const PointT &target, const float distanceTol) {
        // Checking if the current node is inside the BB = distanceTol*2 for all three dimensions
        // If yes then calculating the distance to the target point
        // Then if it's within the distanceTol, adding the current node's id to the list
        if (node != NULL) {
            if (node->isWithinDistanceTo(target, distanceTol) && node->getDistanceTo(target) <= distanceTol)
                ids.indices.push_back(node->id);
            
            // Checking if the left/right branches should be checked for inclusion: checking the split axis coordinate value.
            uint axis = sublevel % 3;
            float point_dimension_length {}, target_dimension_length {};
            
            if (axis == 0)
                point_dimension_length = node->point.x, target_dimension_length = target.x;
            else if (axis == 1)
                point_dimension_length = node->point.y, target_dimension_length = target.y;
            else
                point_dimension_length = node->point.z, target_dimension_length = target.z;
            
            if (point_dimension_length > target_dimension_length - distanceTol)
                helperSearch(ids, sublevel+1, node->left, target, distanceTol);
            if (point_dimension_length < target_dimension_length + distanceTol)
                helperSearch(ids, sublevel+1, node->right, target, distanceTol);
        }
    }
    
    // return a list of point ids in the tree that are within distance of target
    pcl::PointIndices search(const PointT &target, const float distanceTol) {
        pcl::PointIndices ids;
        uint sublevel {0};
        
        helperSearch(ids, sublevel, root, target, distanceTol);
        
        return ids;
    }
};


template<typename PointT>
void processPoint(const size_t &i, pcl::PointIndices &cluster, std::vector<bool> &processed, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, const float &distanceTol) {
   
    processed[i] = true;
    cluster.indices.push_back(i);
    
    pcl::PointIndices nearest = tree->search(cloud->points[i], distanceTol);
    
    for (auto &id: nearest.indices)
        if (!processed[id])
            processPoint(id, cluster, processed, cloud, tree, distanceTol);
}


template<typename PointT>
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, int minSize, int maxSize, const float distanceTol) {
    std::vector<pcl::PointIndices> clusters;
    std::vector<bool> processed (cloud->points.size(), false);
    
    for (size_t i=0; i < cloud->points.size(); i++) {
        if (!processed[i]) {
            pcl::PointIndices cluster;
            processPoint(i, cluster, processed, cloud, tree, distanceTol);
            
            if (cluster.indices.size() >= minSize && cluster.indices.size() <= maxSize)
                clusters.push_back(cluster);
            else {
                std::string cluster_str;
                for (auto &i: cluster.indices)
                    cluster_str += " " + std::to_string(i);
                //std::cout << "Cluster" << cluster_str << " is out of the size limit bounds (" << minSize << ", " << maxSize << "). Skipping..." << std::endl;
            }
        }
    }
 
    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
                                                                                          int minSize, int maxSize, bool useCustomAlgorithm) {
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> cluster_indices;
    
    if (useCustomAlgorithm) {
        std::cout << "Using custom clustering algorithm" << std::endl;
        // Creating the KdTree object for the obstacles point cloud
        KdTree<PointT>* tree = new KdTree<PointT>;
  
        int it = 0;
        for (int i=0; i<cloud->points.size(); i++) 
            tree->insert(tree->root, it, cloud->points[i], i);
        
        // Creating the vector of cluster indices
        cluster_indices = euclideanCluster<PointT>(cloud, tree, minSize, maxSize, clusterTolerance);
    
    } else {
        // Creating the KdTree object for the obstacles point cloud
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);
        
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (clusterTolerance);
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);
    }
    
    for (const auto& cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        clusters.push_back(cloud_cluster);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


/*#################
 * BOUNDING BOXES
 *#################*/
 
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {

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

// Minimum oriented boundig box
// contains quaternion member that allows Z-axis rotations
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster) {
    // Source used: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
    // Find bounding box for one of the clusters
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    
    // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    typename pcl::PointCloud<PointT>::Ptr cloudPCAprojection (new pcl::PointCloud<PointT>);
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *cloudPCAprojection);
    Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  // This line is necessary for proper orientation in some cases.
                                                                                    // The numbers come out the same without it, but
                                                                                    // the signs are different and the box doesn't get correctly oriented in some cases.
    // Eigen::Matrix3f eigenValuesPCA = pca.getEigenValues();
    
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    
    BoxQ box;
    box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    box.bboxQuaternion = bboxQuaternion;
    box.cube_length = fabs(maxPoint.x - minPoint.x);
    box.cube_width = fabs(maxPoint.y - minPoint.y);
    box.cube_height = fabs(maxPoint.z - minPoint.z);

    return box;
}

/*#######################
 * POINT CLOUD MANAGEMENT
 *#######################*/

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}