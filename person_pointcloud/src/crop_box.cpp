#include <ros/ros.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <pcl/filters/crop_box.h>



using namespace std;

//create class to store information on coordinates
class person_coordinates {
  public:
    float X;
    float Y;
    float Z;

    void msgCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

};
void person_coordinates::msgCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
        X = msg->data[0];
        Y = msg->data[1];
        Z = msg->data[2];
}

//create class to 
class cloud
{

public:

pcl::PCLPointCloud2Ptr C;

cloud(): C(new pcl::PCLPointCloud2)
    {

    }

        void cloud_cb(const pcl::PCLPointCloud2Ptr& point_cloudJ1);
};

void cloud::cloud_cb(const pcl::PCLPointCloud2Ptr& point_cloudJ1)
{

    C = point_cloudJ1;

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "crop_box");
  ros::NodeHandle nh;
  ros::Rate r(15);
  person_coordinates coords;
  cloud cloud1;
  cloud cloud2;
  cloud cloud3;
  cloud cloud4;
  cloud cloud5;
  cloud cloud6;

  // Subscribe to coordinate of person
  ros::Subscriber sub = nh.subscribe("/coordinates_of_person", 1, &person_coordinates::msgCallback, &coords);

  // Subscribe to the point clouds from each jetson(world frame)
  ros::Subscriber sub1 = nh.subscribe ("/master/jetson1/kinect_decomp", 1, &cloud::cloud_cb, &cloud1);
  ros::Subscriber sub2 = nh.subscribe ("/master/jetson2/kinect_decomp", 1, &cloud::cloud_cb, &cloud2);
  ros::Subscriber sub3 = nh.subscribe ("/master/jetson3/kinect_decomp", 1, &cloud::cloud_cb, &cloud3);
  ros::Subscriber sub4 = nh.subscribe ("/master/jetson4/kinect_decomp", 1, &cloud::cloud_cb, &cloud4);
  ros::Subscriber sub5 = nh.subscribe ("/master/jetson5/kinect_decomp", 1, &cloud::cloud_cb, &cloud5);
  ros::Subscriber sub6 = nh.subscribe ("/master/jetson6/kinect_decomp", 1, &cloud::cloud_cb, &cloud6);

  // Publish
  ros::Publisher pubJ1 = nh.advertise<pcl::PCLPointCloud2> ("/crop_box/jetson1", 1);
  ros::Publisher pubJ2 = nh.advertise<pcl::PCLPointCloud2> ("/crop_box/jetson2", 1);
  ros::Publisher pubJ3 = nh.advertise<pcl::PCLPointCloud2> ("/crop_box/jetson3", 1);
  ros::Publisher pubJ4 = nh.advertise<pcl::PCLPointCloud2> ("/crop_box/jetson4", 1);
  ros::Publisher pubJ5 = nh.advertise<pcl::PCLPointCloud2> ("/crop_box/jetson5", 1);
  ros::Publisher pubJ6 = nh.advertise<pcl::PCLPointCloud2> ("/crop_box/jetson6", 1);


while (ros::ok())
{
  //Filter all the point clouds
  pcl::PCLPointCloud2 cloud_filtered1;
  pcl::CropBox<pcl::PCLPointCloud2> boxFilter1;
  boxFilter1.setMin(Eigen::Vector4f(coords.X-0.6, coords.Y-0.6, 0.1, 1.0));
  boxFilter1.setMax(Eigen::Vector4f(coords.X+0.6, coords.Y+0.6, 2.0, 1.0));
  boxFilter1.setInputCloud(cloud1.C);
  boxFilter1.filter(cloud_filtered1);

  pcl::PCLPointCloud2 cloud_filtered2;
  pcl::CropBox<pcl::PCLPointCloud2> boxFilter2;
  boxFilter2.setMin(Eigen::Vector4f(coords.X-0.6, coords.Y-0.6, 0.1, 1.0));
  boxFilter2.setMax(Eigen::Vector4f(coords.X+0.6, coords.Y+0.6, 2.0, 1.0));
  boxFilter2.setInputCloud(cloud2.C);
  boxFilter2.filter(cloud_filtered2);

  pcl::PCLPointCloud2 cloud_filtered3;
  pcl::CropBox<pcl::PCLPointCloud2> boxFilter3;
  boxFilter3.setMin(Eigen::Vector4f(coords.X-0.6, coords.Y-0.6, 0.1, 1.0));
  boxFilter3.setMax(Eigen::Vector4f(coords.X+0.6, coords.Y+0.6, 2.0, 1.0));
  boxFilter3.setInputCloud(cloud3.C);
  boxFilter3.filter(cloud_filtered3);

  pcl::PCLPointCloud2 cloud_filtered4;
  pcl::CropBox<pcl::PCLPointCloud2> boxFilter4;
  boxFilter4.setMin(Eigen::Vector4f(coords.X-0.6, coords.Y-0.6, 0.1, 1.0));
  boxFilter4.setMax(Eigen::Vector4f(coords.X+0.6, coords.Y+0.6, 2.0, 1.0));
  boxFilter4.setInputCloud(cloud4.C);
  boxFilter4.filter(cloud_filtered4);

  pcl::PCLPointCloud2 cloud_filtered5;
  pcl::CropBox<pcl::PCLPointCloud2> boxFilter5;
  boxFilter5.setMin(Eigen::Vector4f(coords.X-0.6, coords.Y-0.6, 0.1, 1.0));
  boxFilter5.setMax(Eigen::Vector4f(coords.X+0.6, coords.Y+0.6, 2.0, 1.0));
  boxFilter5.setInputCloud(cloud5.C);
  boxFilter5.filter(cloud_filtered5);

  pcl::PCLPointCloud2 cloud_filtered6;
  pcl::CropBox<pcl::PCLPointCloud2> boxFilter6;
  boxFilter6.setMin(Eigen::Vector4f(coords.X-0.6, coords.Y-0.6, 0.1, 1.0));
  boxFilter6.setMax(Eigen::Vector4f(coords.X+0.6, coords.Y+0.6, 2.0, 1.0));
  boxFilter6.setInputCloud(cloud6.C);
  boxFilter6.filter(cloud_filtered6);

  // Publish the filtered point clouds
  pubJ1.publish (cloud_filtered1);
  pubJ2.publish (cloud_filtered2);
  pubJ3.publish (cloud_filtered3);
  pubJ4.publish (cloud_filtered4);
  pubJ5.publish (cloud_filtered5);
  pubJ6.publish (cloud_filtered6);



  // Spin
  ros::spinOnce();
  r.sleep();
}
}
