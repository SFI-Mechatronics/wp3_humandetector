#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include<iostream>

using namespace std;

//create dummies to save last iterations values
float array_dummy0 = 0.0;
float array_dummy1 = 0.0;
float array_dummy2 = 0.0;
int dummy = 0;
float JX1temp = 0;
float JX2temp = 0;
float JX3temp = 0;
float JX4temp = 0;
float JX5temp = 0;
float JX6temp = 0;

//create class to stor data from subscription
class coordinates {
  public:
    float X;
    float Y;
    float Z;

    void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

};

void coordinates::msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
        X = msg->bounding_boxes[0].Xg;
        Y = msg->bounding_boxes[0].Yg;
        Z = msg->bounding_boxes[0].Zg;
}



int main( int argc, char *argv[])
{
  ros::init(argc, argv, "sensor_fusion");
  ros::NodeHandle n;
  ros::Rate r(15);

  // create coordinates for each node
  coordinates coords_J1;
  coordinates coords_J2;
  coordinates coords_J3;
  coordinates coords_J4;
  coordinates coords_J5;
  coordinates coords_J6;


 // subscribe to each node
  ros::Subscriber sub_J1 = n.subscribe("/jetson1_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J1);
  ros::Subscriber sub_J2 = n.subscribe("/jetson2_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J2);
  ros::Subscriber sub_J3 = n.subscribe("/jetson3_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J3);
  ros::Subscriber sub_J4 = n.subscribe("/jetson4_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J4);
  ros::Subscriber sub_J5 = n.subscribe("/jetson5_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J5);
  ros::Subscriber sub_J6 = n.subscribe("/jetson6_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J6);

 // create publisher for mean coordinates
  ros::Publisher pub_coord = n.advertise<std_msgs::Float32MultiArray>("coordinates_of_person", 1);

  while(ros::ok())
{

std_msgs::Float32MultiArray array;

array.data.resize(3);


float XJ1;
float YJ1;
float ZJ1;
float XJ2;
float YJ2;
float ZJ2;
float XJ3;
float YJ3;
float ZJ3;
float XJ4;
float YJ4;
float ZJ4;
float XJ5;
float YJ5;
float ZJ5;
float XJ6;
float YJ6;
float ZJ6;
int i = 0;
cout << "x1: " << coords_J1.X << endl;
cout << "temp: " << JX1temp << endl;

//here we set the coordinates to zero if it we have a bad measurement
if(coords_J1.X-JX1temp == 0.0)
{
    XJ1 = 0.0;
    YJ1 = 0.0;
    ZJ1 = 0.0;

}
else if(abs(coords_J1.X-JX1temp) > 500.0)
{
    XJ1 = 0.0;
    YJ1 = 0.0;
    ZJ1 = 0.0;
}
else
{
    XJ1 = coords_J1.X;
    YJ1 = coords_J1.Y;
    ZJ1 = coords_J1.Z;
    i++;
}


if(coords_J2.X-JX2temp == 0.0)
{
    XJ2 = 0.0;
    YJ2 = 0.0;
    ZJ2 = 0.0;

}
else if(abs(coords_J2.X-JX2temp) > 500.0)
{
    XJ2 = 0.0;
    YJ2 = 0.0;
    ZJ2 = 0.0;
}
else
{
    XJ2 = coords_J2.X;
    YJ2 = coords_J2.Y;
    ZJ2 = coords_J2.Z;
    i++;
}

if(coords_J3.X-JX3temp == 0.0)
{
    XJ3 = 0.0;
    YJ3 = 0.0;
    ZJ3 = 0.0;

}
else if(abs(coords_J3.X-JX3temp) > 500.0)
{
    XJ3 = 0.0;
    YJ3 = 0.0;
    ZJ3 = 0.0;
}
else
{
    XJ3 = coords_J3.X;
    YJ3 = coords_J3.Y;
    ZJ3 = coords_J3.Z;
    i++;
}

if(coords_J4.X-JX4temp == 0.0)
{
    XJ4 = 0.0;
    YJ4 = 0.0;
    ZJ4 = 0.0;

}
else if(abs(coords_J4.X-JX4temp) > 500.0)
{
    XJ4 = 0.0;
    YJ4 = 0.0;
    ZJ4 = 0.0;
}
else
{
    XJ4 = coords_J4.X;
    YJ4 = coords_J4.Y;
    ZJ4 = coords_J4.Z;
    i++;
}

if(coords_J5.X-JX5temp == 0.0)
{
    XJ5 = 0.0;
    YJ5 = 0.0;
    ZJ5 = 0.0;

}
else if(abs(coords_J5.X-JX5temp) > 500.0)
{
    XJ5 = 0.0;
    YJ5 = 0.0;
    ZJ5 = 0.0;
}
else
{
    XJ5 = coords_J5.X;
    YJ5 = coords_J5.Y;
    ZJ5 = coords_J5.Z;
    i++;
}

if(coords_J6.X-JX6temp == 0.0)
{
    XJ6 = 0.0;
    YJ6 = 0.0;
    ZJ6 = 0.0;

}
else if(abs(coords_J6.X-JX6temp) > 500.0)
{
    XJ6 = 0.0;
    YJ6 = 0.0;
    ZJ6 = 0.0;
}
else
{
    XJ6 = coords_J6.X;
    YJ6 = coords_J6.Y;
    ZJ6 = coords_J6.Z;
    i++;
}

//make arrays to store all the X-, Y- and Z-coordinates
float arrayX[6]={0.0};
arrayX[0] = XJ1;
arrayX[1] = XJ2;
arrayX[2] = XJ3;
arrayX[3] = XJ4;
arrayX[4] = XJ5;
arrayX[5] = XJ6;

float arrayY[6]={0.0};
arrayY[0] = YJ1;
arrayY[1] = YJ2;
arrayY[2] = YJ3;
arrayY[3] = YJ4;
arrayY[4] = YJ5;
arrayY[5] = YJ6;

float arrayZ[6]={0.0};
arrayZ[0] = ZJ1;
arrayZ[1] = ZJ2;
arrayZ[2] = ZJ3;
arrayZ[3] = ZJ4;
arrayZ[4] = ZJ5;
arrayZ[5] = ZJ6;

// arrange the arrays in increasing order
for(int j=0; j<6; j++)
        {
                for(int k=j+1; k<6; k++)
                {
                        if(arrayX[k] < arrayX[j])
                        {
                        int temp = arrayX[j];
                        arrayX[j] = arrayX[k];
                        arrayX[k] = temp;
                        }

                        if(arrayY[k] < arrayY[j])
                        {
                        int temp = arrayY[j];
                        arrayY[j] = arrayY[k];
                        arrayY[k] = temp;
                        }

                        if(arrayZ[k] < arrayZ[j])
                        {
                        int temp = arrayZ[j];
                        arrayZ[j] = arrayZ[k];
                        arrayZ[k] = temp;
                        }
                }
        }

float sumX = 0.0;
float sumY = 0.0;
float sumZ = 0.0;
int a = 1;

// add all the non-zero values and divide by the number of non-zero values to get the meah value. If detection is lost for brief moments, keep the detection where the last detection was
if(i > 0)
{
    for(int j=6-i; j<6; j++)
    {
        sumX=arrayX[j] + sumX;
        sumY=arrayY[j] + sumY;
        sumZ=arrayZ[j] + sumZ;

    }
    array.data[0] = (sumX / i) / 1000;
    array.data[1] = (sumY / i) / 1000;
    array.data[2] = (sumZ / i) / 1000;
}
else if(dummy == 1)
{
    array.data[0] = array_dummy0;
    array.data[1] = array_dummy1;
    array.data[2] = array_dummy2;
    dummy = 2;
}
else if(dummy == 2)
{
    array.data[0] = array_dummy0;
    array.data[1] = array_dummy1;
    array.data[2] = array_dummy2;
    dummy = 3;
}
else if(dummy == 3)
{
    array.data[0] = array_dummy0;
    array.data[1] = array_dummy1;
    array.data[2] = array_dummy2;
    dummy = 4;
}
else if(dummy == 4)
{
    array.data[0] = array_dummy0;
    array.data[1] = array_dummy1;
    array.data[2] = array_dummy2;
    dummy = 0;
}
else
{
    array.data[0] = 0.0;
    array.data[1] = 0.0;
    array.data[2] = 0.0;
    dummy = 0;
}

cout << "arr1: " << arrayX[0] << endl;
cout << "arr2: " << arrayX[1] << endl;
cout << "arr3: " << arrayX[2] << endl;
cout << "arr4: " << arrayX[3] << endl;
cout << "arr5: " << arrayX[4] << endl;
cout << "arr6: " << arrayX[5] << endl;
cout << "sumX: " << sumX << endl;
cout << "i: " << i << endl;
cout << "-------" << endl;





//publish array
pub_coord.publish(array);


array_dummy0 = array.data[0];
array_dummy1 = array.data[1];
array_dummy2 = array.data[2];

JX1temp = coords_J1.X;
JX2temp = coords_J2.X;
JX3temp = coords_J3.X;
JX4temp = coords_J4.X;
JX5temp = coords_J5.X;
JX6temp = coords_J6.X;

ros::spinOnce();
r.sleep();

}

}
