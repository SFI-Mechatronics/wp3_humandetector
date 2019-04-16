#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include<iostream>

using namespace std;

//Create class to store the information on coordinates
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


int main( int argc, char** argv )
{
  ros::init(argc, argv, "one_cyl");
  ros::NodeHandle n;
  ros::Rate r(15);
  person_coordinates coords;

  //Subscribe to the topic with the coordinates of the person
  ros::Subscriber sub = n.subscribe("/coordinates_of_person", 1, &person_coordinates::msgCallback, &coords);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set the shape to a cylinder
  uint32_t shape = visualization_msgs::Marker::CYLINDER;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker

    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type to the cylinder
    marker.type = shape;

    // Add the marker
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker
    marker.pose.position.x = coords.X;
    marker.pose.position.y = coords.Y;
    marker.pose.position.z = coords.Z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 2.0;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration(0.07);

    // Publish the marker
    if(coords.X != 0.0)
    {
        marker_pub.publish(marker);
    }
    cout << "X: " << coords.X << endl;
    cout << "Y: " << coords.Y << endl;
    cout << "Z: " << coords.Z << endl;


    ros::spinOnce();
    r.sleep();
  }
}
