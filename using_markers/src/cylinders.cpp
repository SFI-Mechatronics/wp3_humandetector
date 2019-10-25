#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include<iostream>

using namespace std;

//create class to store information on coordinates of persons(max 5)
class coordinates {
  public:
    float X[5] = {0.0};
    float Y[5] = {0.0};
    float Z[5] = {0.0};
    float Ymax[5] = {0.0};
    float Ymin[5] = {0.0};


    void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

};

//Create class to store number of person in the image
class objectFound {
  public:
    int num;

    void objCallback(const std_msgs::Int8::ConstPtr& msg);

};

void objectFound::objCallback(const std_msgs::Int8::ConstPtr& msg)
{
        num = msg->data;
}


void coordinates::msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
        //std::cout<<"X:" << msg->bounding_boxes[0].X <<std::endl;


        for(int i = 0; i < msg->bounding_boxes[0].num; i++)
        {
        X[i] = msg->bounding_boxes[i].Xg;
        Y[i] = msg->bounding_boxes[i].Yg;
        Z[i] = msg->bounding_boxes[i].Zg;
        Ymax[i] = msg->bounding_boxes[i].Ymax;
        Ymin[i] = msg->bounding_boxes[i].Ymin;
        }
}


int main( int argc, char *argv[])
{
  ros::init(argc, argv, "cylinder");
  ros::NodeHandle n;
  ros::Rate r(15);
  
  //create coordinates and objectsFound for each of the six nodes
  coordinates coords_J1;
  objectFound obj_J1;
  coordinates coords_J2;
  objectFound obj_J2;
  coordinates coords_J3;
  objectFound obj_J3;
  coordinates coords_J4;
  objectFound obj_J4;
  coordinates coords_J5;
  objectFound obj_J5;
  coordinates coords_J6;
  objectFound obj_J6;

  //subscribe to all the nodes
  ros::Subscriber sub_J1_box = n.subscribe("/jetson1_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J1);
  ros::Subscriber sub_J1_object = n.subscribe("/jetson1_darknet_ros/found_object/", 1, &objectFound::objCallback, &obj_J1);
  ros::Subscriber sub_J2_box = n.subscribe("/jetson2_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J2);
  ros::Subscriber sub_J2_object = n.subscribe("/jetson2_darknet_ros/found_object/", 1, &objectFound::objCallback, &obj_J2);
  ros::Subscriber sub_J3_box = n.subscribe("/jetson3_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J3);
  ros::Subscriber sub_J3_object = n.subscribe("/jetson3_darknet_ros/found_object/", 1, &objectFound::objCallback, &obj_J3);
  ros::Subscriber sub_J4_box = n.subscribe("/jetson4_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J4);
  ros::Subscriber sub_J4_object = n.subscribe("/jetson4_darknet_ros/found_object/", 1, &objectFound::objCallback, &obj_J4);
  ros::Subscriber sub_J5_box = n.subscribe("/jetson5_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J5);
  ros::Subscriber sub_J5_object = n.subscribe("/jetson5_darknet_ros/found_object/", 1, &objectFound::objCallback, &obj_J5);
  ros::Subscriber sub_J6_box = n.subscribe("/jetson6_darknet_ros/bounding_boxes/", 1, &coordinates::msgCallback, &coords_J6);
  ros::Subscriber sub_J6_object = n.subscribe("/jetson6_darknet_ros/found_object/", 1, &objectFound::objCallback, &obj_J6);

//publish an array of markers for each node
  ros::Publisher pub_J1_cyl = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_J1", 1);
  ros::Publisher pub_J2_cyl = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_J2", 1);
  ros::Publisher pub_J3_cyl = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_J3", 1);
  ros::Publisher pub_J4_cyl = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_J4", 1);
  ros::Publisher pub_J5_cyl = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_J5", 1);
  ros::Publisher pub_J6_cyl = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_J6", 1);


  while (ros::ok())
  {
      // create marker array for node 1 and rezize based on number of persons in picture
      visualization_msgs::MarkerArray cyl_J1;
  int l = 0;
  if(obj_J1.num == 1){
    cyl_J1.markers.resize(1);
    l = 1;
  } else if(obj_J1.num == 2){
      cyl_J1.markers.resize(2);
      l = 2;
  } else if(obj_J1.num == 3) {
      cyl_J1.markers.resize(3);
      l = 3;
  } else if(obj_J1.num == 4) {
      cyl_J1.markers.resize(4);
      l = 4;
  } else if(obj_J1.num == 5) {
      cyl_J1.markers.resize(5);
      l = 5;
  }
  for(int k = 0; k < l; k++)
      {
      cout << "x1" << k << ": " << coords_J1.X[k] << endl;
      cout << "y1" << k << ": " << coords_J1.Y[k] << endl;
      cout << "z1" << k << ": " << coords_J1.Z[k] << endl;

      }

	// for each person found, make a cylinder corresponding to their coordinates
        for(int j = 0; j < obj_J1.num; j++)
        {

            if(coords_J1.Z[j] > 50.0 && coords_J1.Z[j] < 2000.0)
            {
                cyl_J1.markers[j].header.frame_id = "world";
                cyl_J1.markers[j].header.stamp = ros::Time::now();
                cyl_J1.markers[j].id = j+10;
                cyl_J1.markers[j].ns ="cylinders_J1";
                cyl_J1.markers[j].action = visualization_msgs::Marker::ADD;
                cyl_J1.markers[j].type = visualization_msgs::Marker::CYLINDER;

                // pose
                cyl_J1.markers[j].pose.position.x = coords_J1.X[j] / 1000;
                cyl_J1.markers[j].pose.position.y = coords_J1.Y[j] / 1000;
                cyl_J1.markers[j].pose.position.z = coords_J1.Z[j] / 1000;
                cyl_J1.markers[j].pose.orientation.x = 0.0;
                cyl_J1.markers[j].pose.orientation.y = 0.0;
                cyl_J1.markers[j].pose.orientation.z = 0.0;
                cyl_J1.markers[j].pose.orientation.w = 1.0;



                // scale
                cyl_J1.markers[j].scale.x = 1.0;
                cyl_J1.markers[j].scale.y = 1.0;
                cyl_J1.markers[j].scale.z = 2 * coords_J1.Z[j] / 1000;

                // color
                cyl_J1.markers[j].color.r = 1.0f;
                cyl_J1.markers[j].color.g = 0.0f;
                cyl_J1.markers[j].color.b = 0.0f;
                cyl_J1.markers[j].color.a = 0.5;
                cyl_J1.markers[j].lifetime = ros::Duration(0.067);
            }

        }
	//create marker array for node 2
        visualization_msgs::MarkerArray cyl_J2;
      l = 0;
    if(obj_J2.num == 1){
      cyl_J2.markers.resize(1);
      l = 1;
    } else if(obj_J2.num == 2){
        cyl_J2.markers.resize(2);
        l = 2;
    } else if(obj_J2.num == 3) {
        cyl_J2.markers.resize(3);
        l = 3;
    } else if(obj_J2.num == 4) {
        cyl_J2.markers.resize(4);
        l = 4;
    } else if(obj_J2.num == 5) {
        cyl_J2.markers.resize(5);
        l = 5;
    }
    for(int k = 0; k < l; k++)
        {
        cout << "x2" << k << ": " << coords_J2.X[k] << endl;
        cout << "y2" << k << ": " << coords_J2.Y[k] << endl;
        cout << "z2" << k << ": " << coords_J2.Z[k] << endl;

        }


          for(int j = 0; j < obj_J2.num; j++)
          {
            bool equal = false;

		// if another node have already made a cylinder for a specific person, this nodes ignores it
             
            for(int i =0; i < obj_J1.num; i++)
            {
              double Xdiff21[i];
              Xdiff21[i] = abs(coords_J1.X[i] - coords_J2.X[j]);
              double Ydiff21[i];
              Ydiff21[i] = abs(coords_J1.Y[i] - coords_J2.Y[j]);
              if(Xdiff21[i] < 450.0 && Ydiff21[i] < 450.0)
              {
                equal = true;
              }
            }
            if(coords_J2.Z[j] > 50.0 && coords_J2.Z[j] < 2000.0 && equal == false)
            {
              cyl_J2.markers[j].header.frame_id = "world";
              cyl_J2.markers[j].header.stamp = ros::Time::now();
              cyl_J2.markers[j].id = j+20;
              cyl_J2.markers[j].ns ="cylinders_J2";
              cyl_J2.markers[j].action = visualization_msgs::Marker::ADD;
              cyl_J2.markers[j].type = visualization_msgs::Marker::CYLINDER;

              cyl_J2.markers[j].pose.position.x = coords_J2.X[j] / 1000;
              cyl_J2.markers[j].pose.position.y = coords_J2.Y[j] / 1000;
              cyl_J2.markers[j].pose.position.z = coords_J2.Z[j] / 1000;
              cyl_J2.markers[j].pose.orientation.x = 0.0;
              cyl_J2.markers[j].pose.orientation.y = 0.0;
              cyl_J2.markers[j].pose.orientation.z = 0.0;
              cyl_J2.markers[j].pose.orientation.w = 1.0;

              cyl_J2.markers[j].scale.x = 1.0;
              cyl_J2.markers[j].scale.y = 1.0;
              cyl_J2.markers[j].scale.z = 2 * coords_J2.Z[j] / 1000;

              cyl_J2.markers[j].color.r = 0.0f;
              cyl_J2.markers[j].color.g = 1.0f;
              cyl_J2.markers[j].color.b = 0.0f;
              cyl_J2.markers[j].color.a = 0.5;
              cyl_J2.markers[j].lifetime = ros::Duration(0.067);
            }

          }
	// Repeat for the rest of the nodes
      visualization_msgs::MarkerArray cyl_J3;
    l = 0;
  if(obj_J3.num == 1){
    cyl_J3.markers.resize(1);
    l = 1;
  } else if(obj_J3.num == 2){
      cyl_J3.markers.resize(2);
      l = 2;
  } else if(obj_J3.num == 3) {
      cyl_J3.markers.resize(3);
      l = 3;
  } else if(obj_J3.num == 4) {
      cyl_J3.markers.resize(4);
      l = 4;
  } else if(obj_J3.num == 5) {
      cyl_J3.markers.resize(5);
      l = 5;
  }
  for(int k = 0; k < l; k++)
      {
      cout << "x3" << k << ": " << coords_J3.X[k] << endl;
      cout << "y3" << k << ": " << coords_J3.Y[k] << endl;
      cout << "z3" << k << ": " << coords_J3.Z[k] << endl;

      }


        for(int j = 0; j < obj_J3.num; j++)
        {
            bool equal = false;
              for(int i =0; i < obj_J1.num; i++)
              {
              double Xdiff31[i];
              Xdiff31[i] = abs(coords_J1.X[i] - coords_J3.X[j]);
              double Ydiff31[i];
              Ydiff31[i] = abs(coords_J1.Y[i] - coords_J3.Y[j]);
              if(Xdiff31[i] < 450.0 && Ydiff31[i] < 450.0)
              {
                  equal = true;
              }
              }

              for(int i =0; i < obj_J2.num; i++)
              {
              double Xdiff32[i];
              Xdiff32[i] = abs(coords_J2.X[i] - coords_J3.X[j]);
              double Ydiff32[i];
              Ydiff32[i] = coords_J2.Y[i] - coords_J3.Y[j];
              if(Xdiff32[i] < 450.0 && Ydiff32[i] < 450.0)
              {
                  equal = true;
              }
              }

            if(coords_J3.Z[j] > 50.0 && coords_J3.Z[j] < 2000.0 && equal == false)
            {
                cyl_J3.markers[j].header.frame_id = "world";
                cyl_J3.markers[j].header.stamp = ros::Time::now();
                cyl_J3.markers[j].id = j+30;
                cyl_J3.markers[j].ns ="cylinders_J3";
                cyl_J3.markers[j].action = visualization_msgs::Marker::ADD;
                cyl_J3.markers[j].type = visualization_msgs::Marker::CYLINDER;

                
                cyl_J3.markers[j].pose.position.x = coords_J3.X[j] / 1000;
                cyl_J3.markers[j].pose.position.y = coords_J3.Y[j] / 1000;
                cyl_J3.markers[j].pose.position.z = coords_J3.Z[j] / 1000;
                cyl_J3.markers[j].pose.orientation.x = 0.0;
                cyl_J3.markers[j].pose.orientation.y = 0.0;
                cyl_J3.markers[j].pose.orientation.z = 0.0;
                cyl_J3.markers[j].pose.orientation.w = 1.0;



                
                cyl_J3.markers[j].scale.x = 1.0;
                cyl_J3.markers[j].scale.y = 1.0;
                cyl_J3.markers[j].scale.z = 2 * coords_J3.Z[j] / 1000;

                
                cyl_J3.markers[j].color.r = 0.0f;
                cyl_J3.markers[j].color.g = 0.0f;
                cyl_J3.markers[j].color.b = 1.0f;
                cyl_J3.markers[j].color.a = 0.5;
                cyl_J3.markers[j].lifetime = ros::Duration(0.067);
            }

        }


        visualization_msgs::MarkerArray cyl_J4;
      l = 0;
    if(obj_J4.num == 1){
      cyl_J4.markers.resize(1);
      l = 1;
    } else if(obj_J4.num == 2){
        cyl_J4.markers.resize(2);
        l = 2;
    } else if(obj_J4.num == 3) {
        cyl_J4.markers.resize(3);
        l = 3;
    } else if(obj_J4.num == 4) {
        cyl_J4.markers.resize(4);
        l = 4;
    } else if(obj_J4.num == 5) {
        cyl_J4.markers.resize(5);
        l = 5;
    }
    for(int k = 0; k < l; k++)
        {
        cout << "x4" << k << ": " << coords_J4.X[k] << endl;
        cout << "y4" << k << ": " << coords_J4.Y[k] << endl;
        cout << "z4" << k << ": " << coords_J4.Z[k] << endl;

        }


          for(int j = 0; j < obj_J4.num; j++)
          {
              bool equal = false;
                for(int i =0; i < obj_J1.num; i++)
                {
                double Xdiff41[i];
                Xdiff41[i] = abs(coords_J1.X[i] - coords_J4.X[j]);
                double Ydiff41[i];
                Ydiff41[i] = abs(coords_J1.Y[i] - coords_J4.Y[j]);
                if(Xdiff41[i] < 450.0 && Ydiff41[i] < 450.0)
                {
                    equal = true;
                }
                }

                for(int i =0; i < obj_J2.num; i++)
                {
                double Xdiff42[i];
                Xdiff42[i] = abs(coords_J2.X[i] - coords_J4.X[j]);
                double Ydiff42[i];
                Ydiff42[i] = abs(coords_J2.Y[i] - coords_J4.Y[j]);
                if(Xdiff42[i] < 450.0 && Ydiff42[i] < 450.0)
                {
                    equal = true;
                }
                }

                for(int i =0; i < obj_J3.num; i++)
                {
                double Xdiff43[i];
                Xdiff43[i] = abs(coords_J3.X[i] - coords_J4.X[j]);
                double Ydiff43[i];
                Ydiff43[i] = abs(coords_J3.Y[i] - coords_J4.Y[j]);
                if(Xdiff43[i] < 450.0 && Ydiff43[i] < 450.0)
                {
                    equal = true;
                }
                }

                if(coords_J4.Z[j] > 50.0 && coords_J4.Z[j] < 2000.0 && equal == false)
              {
                  cyl_J4.markers[j].header.frame_id = "world";
                  cyl_J4.markers[j].header.stamp = ros::Time::now();
                  cyl_J4.markers[j].id = j+40;
                  cyl_J4.markers[j].ns ="cylinders_J4";
                  cyl_J4.markers[j].action = visualization_msgs::Marker::ADD;
                  cyl_J4.markers[j].type = visualization_msgs::Marker::CYLINDER;

                 
                  cyl_J4.markers[j].pose.position.x = coords_J4.X[j] / 1000;
                  cyl_J4.markers[j].pose.position.y = coords_J4.Y[j] / 1000;
                  cyl_J4.markers[j].pose.position.z = coords_J4.Z[j] / 1000;
                  cyl_J4.markers[j].pose.orientation.x = 0.0;
                  cyl_J4.markers[j].pose.orientation.y = 0.0;
                  cyl_J4.markers[j].pose.orientation.z = 0.0;
                  cyl_J4.markers[j].pose.orientation.w = 1.0;



                  
                  cyl_J4.markers[j].scale.x = 1.0;
                  cyl_J4.markers[j].scale.y = 1.0;
                  cyl_J4.markers[j].scale.z = 2 * coords_J4.Z[j] / 1000;

                  
                  cyl_J4.markers[j].color.r = 0.0f;
                  cyl_J4.markers[j].color.g = 0.0f;
                  cyl_J4.markers[j].color.b = 0.0f;
                  cyl_J4.markers[j].color.a = 0.5;
                  cyl_J4.markers[j].lifetime = ros::Duration(0.067);
              }

          }


          visualization_msgs::MarkerArray cyl_J5;
        l = 0;
      if(obj_J5.num == 1){
        cyl_J5.markers.resize(1);
        l = 1;
      } else if(obj_J5.num == 2){
          cyl_J5.markers.resize(2);
          l = 2;
      } else if(obj_J5.num == 3) {
          cyl_J5.markers.resize(3);
          l = 3;
      } else if(obj_J5.num == 4) {
          cyl_J5.markers.resize(4);
          l = 4;
      } else if(obj_J5.num == 5) {
          cyl_J5.markers.resize(5);
          l = 5;
      }
      for(int k = 0; k < l; k++)
          {
          cout << "x5" << k << ": " << coords_J5.X[k] << endl;
          cout << "y5" << k << ": " << coords_J5.Y[k] << endl;
          cout << "z5" << k << ": " << coords_J5.Z[k] << endl;

          }


            for(int j = 0; j < obj_J5.num; j++)
            {
                bool equal = false;
                  for(int i =0; i < obj_J1.num; i++)
                  {
                  double Xdiff51[i];
                  Xdiff51[i] = abs(coords_J1.X[i] - coords_J5.X[j]);
                  double Ydiff51[i];
                  Ydiff51[i] = abs(coords_J1.Y[i] - coords_J5.Y[j]);
                  if(Xdiff51[i] < 450.0 && Ydiff51[i] < 450.0)
                  {
                      equal = true;
                  }
                  }

                  for(int i =0; i < obj_J2.num; i++)
                  {
                  double Xdiff52[i];
                  Xdiff52[i] = abs(coords_J2.X[i] - coords_J5.X[j]);
                  double Ydiff52[i];
                  Ydiff52[i] = abs(coords_J2.Y[i] - coords_J5.Y[j]);
                  if(Xdiff52[i] < 450.0 && Ydiff52[i] < 450.0)
                  {
                      equal = true;
                  }
                  }

                  for(int i =0; i < obj_J3.num; i++)
                  {
                  double Xdiff53[i];
                  Xdiff53[i] = abs(coords_J3.X[i] - coords_J5.X[j]);
                  double Ydiff53[i];
                  Ydiff53[i] = abs(coords_J3.Y[i] - coords_J5.Y[j]);
                  if(Xdiff53[i] < 450.0 && Ydiff53[i] < 450.0)
                  {
                      equal = true;
                  }
                  }

                  for(int i =0; i < obj_J4.num; i++)
                  {
                  double Xdiff54[i];
                  Xdiff54[i] = abs(coords_J4.X[i] - coords_J5.X[j]);
                  double Ydiff54[i];
                  Ydiff54[i] = abs(coords_J4.Y[i] - coords_J5.Y[j]);
                  if(Xdiff54[i] < 450.0 && Ydiff54[i] < 450.0)
                  {
                      equal = true;
                  }
                  }

                if(coords_J5.Z[j] > 50.0 && coords_J5.Z[j] < 2000.0 && equal == false)
                {
                    cyl_J5.markers[j].header.frame_id = "world";
                    cyl_J5.markers[j].header.stamp = ros::Time::now();
                    cyl_J5.markers[j].id = j+50;
                    cyl_J5.markers[j].ns ="cylinders_J5";
                    cyl_J5.markers[j].action = visualization_msgs::Marker::ADD;
                    cyl_J5.markers[j].type = visualization_msgs::Marker::CYLINDER;

                    
                    cyl_J5.markers[j].pose.position.x = coords_J5.X[j] / 1000;
                    cyl_J5.markers[j].pose.position.y = coords_J5.Y[j] / 1000;
                    cyl_J5.markers[j].pose.position.z = coords_J5.Z[j] / 1000;
                    cyl_J5.markers[j].pose.orientation.x = 0.0;
                    cyl_J5.markers[j].pose.orientation.y = 0.0;
                    cyl_J5.markers[j].pose.orientation.z = 0.0;
                    cyl_J5.markers[j].pose.orientation.w = 1.0;



                    
                    cyl_J5.markers[j].scale.x = 1.0;
                    cyl_J5.markers[j].scale.y = 1.0;
                    cyl_J5.markers[j].scale.z = 2 * coords_J5.Z[j] / 1000;

                    
                    cyl_J5.markers[j].color.r = 1.0f;
                    cyl_J5.markers[j].color.g = 1.0f;
                    cyl_J5.markers[j].color.b = 1.0f;
                    cyl_J5.markers[j].color.a = 0.5;
                    cyl_J5.markers[j].lifetime = ros::Duration(0.067);
                }

            }

            visualization_msgs::MarkerArray cyl_J6;
          l = 0;
        if(obj_J6.num == 1){
          cyl_J6.markers.resize(1);
          l = 1;
        } else if(obj_J6.num == 2){
            cyl_J6.markers.resize(2);
            l = 2;
        } else if(obj_J6.num == 3) {
            cyl_J6.markers.resize(3);
            l = 3;
        } else if(obj_J6.num == 4) {
            cyl_J6.markers.resize(4);
            l = 4;
        } else if(obj_J6.num == 5) {
            cyl_J6.markers.resize(5);
            l = 5;
        }
        for(int k = 0; k < l; k++)
            {
            cout << "x6" << k << ": " << coords_J6.X[k] << endl;
            cout << "y6" << k << ": " << coords_J6.Y[k] << endl;
            cout << "z6" << k << ": " << coords_J6.Z[k] << endl;
            cout << "------" << endl;
            }


              for(int j = 0; j < obj_J6.num; j++)
              {
                  bool equal = false;
                    for(int i =0; i < obj_J1.num; i++)
                    {
                    double Xdiff61[i];
                    Xdiff61[i] = abs(coords_J1.X[i] - coords_J6.X[j]);
                    double Ydiff61[i];
                    Ydiff61[i] = abs(coords_J1.Y[i] - coords_J6.Y[j]);
                    if(Xdiff61[i] < 450.0 && Ydiff61[i] < 450.0)
                    {
                        equal = true;
                    }
                    }

                    for(int i =0; i < obj_J2.num; i++)
                    {
                    double Xdiff62[i];
                    Xdiff62[i] = abs(coords_J2.X[i] - coords_J6.X[j]);
                    double Ydiff62[i];
                    Ydiff62[i] = abs(coords_J2.Y[i] - coords_J6.Y[j]);
                    if(Xdiff62[i] < 450.0 && Ydiff62[i] < 450.0)
                    {
                        equal = true;
                    }
                    }

                    for(int i =0; i < obj_J3.num; i++)
                    {
                    double Xdiff63[i];
                    Xdiff63[i] = abs(coords_J3.X[i] - coords_J6.X[j]);
                    double Ydiff63[i];
                    Ydiff63[i] = abs(coords_J3.Y[i] - coords_J6.Y[j]);
                    if(Xdiff63[i] < 450.0 && Ydiff63[i] < 450.0)
                    {
                        equal = true;
                    }
                    }

                    for(int i =0; i < obj_J4.num; i++)
                    {
                    double Xdiff64[i];
                    Xdiff64[i] = abs(coords_J4.X[i] - coords_J6.X[j]);
                    double Ydiff64[i];
                    Ydiff64[i] = abs(coords_J4.Y[i] - coords_J6.Y[j]);
                    if(Xdiff64[i] < 450.0 && Ydiff64[i] < 450.0)
                    {
                        equal = true;
                    }
                    }

                    for(int i =0; i < obj_J5.num; i++)
                    {
                    double Xdiff65[i];
                    Xdiff65[i] = abs(coords_J5.X[i] - coords_J6.X[j]);
                    double Ydiff65[i];
                    Ydiff65[i] = abs(coords_J5.Y[i] - coords_J6.Y[j]);
                    if(Xdiff65[i] < 450.0 && Ydiff65[i] < 450.0)
                    {
                        equal = true;
                    }
                    }

                  if(coords_J6.Z[j] > 50.0 && coords_J6.Z[j] < 2000.0 && equal == false)
                  {
                      cyl_J6.markers[j].header.frame_id = "world";
                      cyl_J6.markers[j].header.stamp = ros::Time::now();
                      cyl_J6.markers[j].id = j+60;
                      cyl_J6.markers[j].ns ="cylinders_J6";
                      cyl_J6.markers[j].action = visualization_msgs::Marker::ADD;
                      cyl_J6.markers[j].type = visualization_msgs::Marker::CYLINDER;

                      
                      cyl_J6.markers[j].pose.position.x = coords_J6.X[j] / 1000;
                      cyl_J6.markers[j].pose.position.y = coords_J6.Y[j] / 1000;
                      cyl_J6.markers[j].pose.position.z = coords_J6.Z[j] / 1000;
                      cyl_J6.markers[j].pose.orientation.x = 0.0;
                      cyl_J6.markers[j].pose.orientation.y = 0.0;
                      cyl_J6.markers[j].pose.orientation.z = 0.0;
                      cyl_J6.markers[j].pose.orientation.w = 1.0;



                      
                      cyl_J6.markers[j].scale.x = 1.0;
                      cyl_J6.markers[j].scale.y = 1.0;
                      cyl_J6.markers[j].scale.z = 2 * coords_J6.Z[j] / 1000;

                      
                      cyl_J6.markers[j].color.r = 1.0f;
                      cyl_J6.markers[j].color.g = 0.0f;
                      cyl_J6.markers[j].color.b = 1.0f;
                      cyl_J6.markers[j].color.a = 0.5;
                      cyl_J6.markers[j].lifetime = ros::Duration(0.067);
                  }

              }


    ros::spinOnce();

    // Publish the marker arrays
    while (pub_J1_cyl.getNumSubscribers() < 1 && pub_J2_cyl.getNumSubscribers() < 1 && pub_J3_cyl.getNumSubscribers() < 1 && pub_J4_cyl.getNumSubscribers() < 1 && pub_J5_cyl.getNumSubscribers() < 1 && pub_J6_cyl.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    pub_J1_cyl.publish(cyl_J1);
    pub_J2_cyl.publish(cyl_J2);
    pub_J3_cyl.publish(cyl_J3);
    pub_J4_cyl.publish(cyl_J4);
    pub_J5_cyl.publish(cyl_J5);
    pub_J6_cyl.publish(cyl_J6);



    r.sleep();
  }
}
