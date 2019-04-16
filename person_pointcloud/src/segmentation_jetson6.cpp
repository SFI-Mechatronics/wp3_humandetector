#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_J6)
{


  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;


  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_J6, *cloud);

  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2( *cloud, point_cloud);
  pcl::copyPointCloud(point_cloud, *point_cloudPtr);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(point_cloudPtr);

  // Createing vector to store clusters
  std::vector<pcl::PointIndices> cluster_indices;
  ros::NodeHandle n;
  int minPoints;
  n.param("minPoints", minPoints, 100);

  // Clustering
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
  cluster.setClusterTolerance(0.1);
  cluster.setMinClusterSize(minPoints);
  cluster.setMaxClusterSize(2000);
  cluster.setSearchMethod(tree);
  cluster.setInputCloud(point_cloudPtr);
  cluster.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);



  for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin(); i < cluster_indices.end(); i++)
          {
        for (std::vector<int>::const_iterator j = i->indices.begin(); j < i->indices.end(); j++)
                    {
                pcl::PointXYZ point;
                point.x = point_cloudPtr->points[*j].x;
                point.y = point_cloudPtr->points[*j].y;
                point.z = point_cloudPtr->points[*j].z;


                point_cloud_segmented->push_back(point);

            }

    }

  // Convert to ROS data type
  point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;

  pcl::PCLPointCloud2 cloud_filtered;
  pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
    ros::init (argc, argv, "segmentation_jetson6");

    ros::NodeHandle nh;


  // Create ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/crop_box/jetson6", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("person_points/jetson6", 1);

  // Spin
  ros::spin ();
}

