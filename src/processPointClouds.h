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

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
    PointT point;
    int id;
    Node* left;
    Node* right;

    Node(PointT arr, int setId)
    :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

template<typename PointT>
struct KdTree
{
    Node<PointT>* root;

    KdTree()
    : root(NULL)
    {}

    void insertHelper(Node<PointT>* &node, PointT point, uint depth,  int id)
    {
        if(node == NULL)
        {
            node = new Node<PointT>(point, id);
        }
        else
        {
            uint cd = depth % 2;
            if(cd == 0)
            {
                if(node->point.x > point.x)
                {
                    insertHelper(node->left, point, depth+1, id);
                }
                else
                {
                    insertHelper(node->right, point, depth+1, id);
                }
            }
            else
            {
                if(node->point.y > point.y)
                {
                    insertHelper(node->left, point, depth+1, id);
                }
                else
                {
                    insertHelper(node->right, point, depth+1, id);
                }
            }

        }

    }

    void insert(PointT point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        insertHelper(root, point, 0, id);
    }

    void searchHelper(Node<PointT>* &node, std::vector<int>* ids, PointT target, uint depth, float distTol)
    {
        if(node!=NULL)
        {
            float dx = node->point.x - target.x;
            float dy = node->point.y - target.y;
            if((node->point.x >= (target.x - distTol))  && (node->point.x <= (target.x + distTol)) && (node->point.y >= (target.y - distTol)) && (node->point.y <= (target.y + distTol)) )
            {
                float dist = sqrt(dx*dx + dy*dy);
                if (dist <= distTol)
                {
                    ids->push_back(node->id);
                }
            }
            uint cd = depth % 2;
            if(cd == 0)
            {
                if(node->point.x > target.x - distTol)
                {
                    searchHelper(node->left, ids, target, depth+1, distTol);
                }
                if(node->point.x < target.x + distTol)
                {
                    searchHelper(node->right, ids, target, depth+1, distTol);
                }

            }
            else
            {
                if(node->point.y > target.y - distTol)
                {
                    searchHelper(node->left, ids, target, depth+1, distTol);
                }
                if(node->point.y < target.y + distTol)
                {
                    searchHelper(node->right, ids, target, depth+1, distTol);
                }

            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root, &ids, target, 0, distanceTol);
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

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane_algo(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<std::vector<int>> euclideanCluster(const std::vector<PointT>& points, KdTree<PointT>* tree, float distanceTol);

    void clusterHelper(int idx, std::vector<bool>& processed, std::vector<int>& cluster, const std::vector<PointT>& points, KdTree<PointT>* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering_algo(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */
