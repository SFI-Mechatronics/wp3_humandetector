/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 *
 *  Modified on: May 20, 2018
 *      Authors: Alejandro Díaz, Adrian Romero and Gonzalo Nuño
 *    Institute: UPM, Universidad Politécnica de Madrid
 */

// YOLO object detector
#include "darknet_ros/YoloObjectDetector.hpp"
//
#include <cmath>
// Check for xServer
#include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

// Initialize arrays to store last steps coordinates
float Xg_temp[10]={0}; //uia
float Yg_temp[10]={0}; //uia

namespace darknet_ros 
{
   char *cfg;
   char *weights;
   char *data;
   char **detectionNames;

   YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
       : nodeHandle_(nh),
         imageTransport_(nodeHandle_),
         numClasses_(0),
         classLabels_(0),
         rosBoxes_(0),
         rosBoxCounter_(0),
         imagergb_sub(imageTransport_,"/camera_crop/image_rect_color",1),       //For depth inclussion
         imagedepth_sub(imageTransport_,"/jetson/sd/image_depth",1),   //For depth inclussion
         sync_1(MySyncPolicy_1(5), imagergb_sub, imagedepth_sub)        //For depth inclussion

   {
      ROS_INFO("[YoloObjectDetector] Node started.");

      // Read parameters from config file.
      if (!readParameters())
      {
         ros::requestShutdown();
      }
      init();
   }

   YoloObjectDetector::~YoloObjectDetector()
   {
      {
         boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
         isNodeRunning_ = false;
      }
   yoloThread_.join();
   }

   bool YoloObjectDetector::readParameters()
   {
      // Load common parameters.
      nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
      nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
      nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

      // Check if Xserver is running on Linux.
      if (XOpenDisplay(NULL))
      {
         // Do nothing!
         ROS_INFO("[YoloObjectDetector] Xserver is running.");
      }
      else
      {
         ROS_INFO("[YoloObjectDetector] Xserver is not running.");
         viewImage_ = false;
      }

      // Set vector sizes.
      nodeHandle_.param("yolo_model/detection_classes/names", classLabels_, std::vector<std::string>(0));
      numClasses_ = classLabels_.size();
      rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
      rosBoxCounter_ = std::vector<int>(numClasses_);

      return true;
   }

   void YoloObjectDetector::init()
   {
      ROS_INFO("[YoloObjectDetector] init().");

      // Initialize deep network of darknet.
      std::string weightsPath;
      std::string configPath;
      std::string dataPath;
      std::string configModel;
      std::string weightsModel;

      // Threshold of object detection.
      float thresh;
      nodeHandle_.param("threshold", thresh, (float) 0.3);

      // Path to weights file.
      nodeHandle_.param("yolo_model/weight_file/name", weightsModel, std::string("yolov2-tiny.weights"));
      nodeHandle_.param("weights_path", weightsPath, std::string("/default"));
      weightsPath += "/" + weightsModel;
      weights = new char[weightsPath.length() + 1];
      strcpy(weights, weightsPath.c_str());

      // Path to config file.
      nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("yolov2-tiny.cfg"));
      nodeHandle_.param("config_path", configPath, std::string("/default"));
      configPath += "/" + configModel;
      cfg = new char[configPath.length() + 1];
      strcpy(cfg, configPath.c_str());

      // Path to data folder.
      dataPath = darknetFilePath_;
      dataPath += "/data";
      data = new char[dataPath.length() + 1];
      strcpy(data, dataPath.c_str());

      // Get classes.
      detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * sizeof(char*));
      for (int i = 0; i < numClasses_; i++)
      {
          detectionNames[i] = new char[classLabels_[i].length() + 1];
          strcpy(detectionNames[i], classLabels_[i].c_str());
      }

      // Load network.
      setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_, 0, 0, 1, 0.5, 0, 0, 0, 0);
      yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);

      // Initialize publisher and subscriber.
      std::string cameraTopicName;
      int cameraQueueSize;
      std::string objectDetectorTopicName;
      int objectDetectorQueueSize;
      bool objectDetectorLatch;
      std::string boundingBoxesTopicName;
      int boundingBoxesQueueSize;
      bool boundingBoxesLatch;
      std::string detectionImageTopicName;
      int detectionImageQueueSize;
      bool detectionImageLatch;
      //AAA 2019
      std::string poseArrayTopicName;

      std::string depthTopicName;        //For depth inclussion
      int depthQueueSize;                //For depth inclussion

      nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/camera/image_raw"));
      nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);

      nodeHandle_.param("subscribers/camera_depth/topic", depthTopicName, std::string("/depth/image_raw"));   //For depth inclussion
      nodeHandle_.param("subscribers/camera_depth/queue_size", depthQueueSize, 1);                            //For depth inclussion

      nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName, std::string("found_object"));
      nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
      nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);

      nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName, std::string("bounding_boxes"));
      nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
      nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);

      nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName, std::string("detection_image"));
      nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
      nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);

      //AAA 2019
      nodeHandle_.param("publishers/poseArray/topic", poseArrayTopicName, std::string("detection_poseArray"));

      sync_1.registerCallback(boost::bind(&YoloObjectDetector::cameraCallback,this,_1,_2));   //For depth inclussion

      objectPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(objectDetectorTopicName, objectDetectorQueueSize, objectDetectorLatch);
      boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
      detectionImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName, detectionImageQueueSize, detectionImageLatch);

      // AAA 2019
      poseArrayPublisher_ = nodeHandle_.advertise<geometry_msgs::PoseArray>(poseArrayTopicName, 1, false);

      // Action servers.
      std::string checkForObjectsActionName;
      nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName, std::string("check_for_objects"));
      checkForObjectsActionServer_.reset(new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));
      checkForObjectsActionServer_->registerGoalCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));
      checkForObjectsActionServer_->registerPreemptCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this));
      checkForObjectsActionServer_->start();
   }

   void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& msgdepth)
   {

      ROS_DEBUG("[YoloObjectDetector] USB image received.");
      cv_bridge::CvImagePtr cam_image;
      cv_bridge::CvImageConstPtr cam_depth;

      //if (msgdepth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      //   ROS_INFO("32FC1");
      //else if (msgdepth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      //   ROS_INFO("16UC1");

      try
      {
         cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
         cam_depth = cv_bridge::toCvCopy(msgdepth, sensor_msgs::image_encodings::TYPE_32FC1);
		 //cam_depth = cv_bridge::toCvCopy(msgdepth, sensor_msgs::image_encodings::MONO8);

         imageHeader_ = msg->header;
      }

      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
    

      if (cam_image)
      {
         {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            camImageCopy_ = cam_image->image.clone();
         }
         {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
            imageStatus_ = true;
         }
         frameWidth_ = cam_image->image.size().width;
         frameHeight_ = cam_image->image.size().height;
      }

      if (cam_depth)
      {
         DepthImageCopy_ = cam_depth->image.clone();
      }

      return;
   }

   void YoloObjectDetector::checkForObjectsActionGoalCB()
   {
      ROS_DEBUG("[YoloObjectDetector] Start check for objects action.");

      boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> imageActionPtr = checkForObjectsActionServer_->acceptNewGoal();
      sensor_msgs::Image imageAction = imageActionPtr->image;

      cv_bridge::CvImagePtr cam_image;

      try
      {
         cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8);
      }
      
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }

      if (cam_image)
      {
         {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            camImageCopy_ = cam_image->image.clone();
         }
         {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexActionStatus_);
            actionId_ = imageActionPtr->id;
         }
         {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
            imageStatus_ = true;
         }
         frameWidth_ = cam_image->image.size().width;
         frameHeight_ = cam_image->image.size().height;
      }
      return;
   }

   void YoloObjectDetector::checkForObjectsActionPreemptCB()
   {
      ROS_DEBUG("[YoloObjectDetector] Preempt check for objects action.");
      checkForObjectsActionServer_->setPreempted();
   }

   bool YoloObjectDetector::isCheckingForObjects() const
   {
      return (ros::ok() && checkForObjectsActionServer_->isActive() && !checkForObjectsActionServer_->isPreemptRequested());
   }

   bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage)
   {
      if (detectionImagePublisher_.getNumSubscribers() < 1)
         return false;

      cv_bridge::CvImage cvImage;
      cvImage.header.stamp = ros::Time::now();
      cvImage.header.frame_id = "detection_image";
      cvImage.encoding = sensor_msgs::image_encodings::BGR8;
      cvImage.image = detectionImage;
      detectionImagePublisher_.publish(*cvImage.toImageMsg());

      ROS_DEBUG("Detection image has been published.");
      return true;
   }

   //double YoloObjectDetector::getWallTime()
   //{
      //struct timeval time;
      //if (gettimeofday(&time, NULL))
      //{
         //return 0;
      //}
      //return (double) time.tv_sec + (double) time.tv_usec * .000001;
   //}

   int YoloObjectDetector::sizeNetwork(network *net)
   {
      int i;
      int count = 0;
      for(i = 0; i < net->n; ++i)
      {
         layer l = net->layers[i];
         if(l.type == YOLO || l.type == REGION || l.type == DETECTION)
         {
            count += l.outputs;
         }
      }
      return count;
   }

   void YoloObjectDetector::rememberNetwork(network *net)
   {
      int i;
      int count = 0;
      for(i = 0; i < net->n; ++i)
      {
         layer l = net->layers[i];
         if(l.type == YOLO || l.type == REGION || l.type == DETECTION)
         {
            memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
            count += l.outputs;
         }
      }
   }

   detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes)
   {
      int i, j;
      int count = 0;
      fill_cpu(demoTotal_, 0, avg_, 1);

      for(j = 0; j < demoFrame_; ++j)
      {
         axpy_cpu(demoTotal_, 1./demoFrame_, predictions_[j], 1, avg_, 1);
      }

      for(i = 0; i < net->n; ++i)
      {
         layer l = net->layers[i];
         if(l.type == YOLO || l.type == REGION || l.type == DETECTION)
         {
            memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
            count += l.outputs;
         }
      }
      detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
      return dets;
   }

   void *YoloObjectDetector::detectInThread()
   {
      running_ = 1;
      float nms = .4;

      layer l = net_->layers[net_->n - 1];
      float *X = buffLetter_[(buffIndex_ + 2) % 3].data;
      float *prediction = network_predict(net_, X);

      rememberNetwork(net_);
      detection *dets = 0;
      int nboxes = 0;
      dets = avgPredictions(net_, &nboxes);

      if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

      if (enableConsoleOutput_)
      {
         printf("\033[2J");
         printf("\033[1;1H");
         printf("\nFPS:%.1f\n",fps_);
         printf("Objects:\n\n");
      }
      image display = buff_[(buffIndex_+2) % 3];
      draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);

      // Extract the bounding boxes and send them to ROS
      int i, j;
      int count = 0;
      for (i = 0; i < nboxes; ++i)
      {
         float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
         float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
         float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
         float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

         if (xmin < 0)
            xmin = 0;
         if (ymin < 0)
            ymin = 0;
         if (xmax > 1)
            xmax = 1;
         if (ymax > 1)
            ymax = 1;

         // Iterate through possible boxes and collect the bounding boxes
         for (j = 0; j < demoClasses_; ++j)
         {
            if (dets[i].prob[j])
            {
               float x_center = (xmin + xmax) / 2;
               float y_center = (ymin + ymax) / 2;
               float BoundingBox_width = xmax - xmin;
               float BoundingBox_height = ymax - ymin;

               // Define bounding box - BoundingBox must be 1% size of frame (3.2x2.4 pixels)
               if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01)
               {
                  roiBoxes_[count].x = x_center;
                  roiBoxes_[count].y = y_center;
                  roiBoxes_[count].w = BoundingBox_width;
                  roiBoxes_[count].h = BoundingBox_height;
                  roiBoxes_[count].Class = j;
                  roiBoxes_[count].prob = dets[i].prob[j];
                  count++;
               }
            }
         }
      }

      // Create array to store found bounding boxes
      // If no object detected, make sure that ROS knows that num = 0
      if (count == 0) 
      {
         roiBoxes_[0].num = 0;
      }
      else
      {
         roiBoxes_[0].num = count;
      }

      free_detections(dets, nboxes);
      demoIndex_ = (demoIndex_ + 1) % demoFrame_;
      running_ = 0;
      return 0;
   }

   void *YoloObjectDetector::fetchInThread()
   {
      IplImage* ROS_img = getIplImage();
      ipl_into_image(ROS_img, buff_[buffIndex_]);
      {
         boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
         buffId_[buffIndex_] = actionId_;
      }
      rgbgr_image(buff_[buffIndex_]);
      letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
      return 0;
   }

   void *YoloObjectDetector::displayInThread(void *ptr)
   {
      show_image_cv(buff_[(buffIndex_ + 1)%3], "YOLO V3", ipl_);
      int c = cvWaitKey(waitKeyDelay_);
      if (c != -1) c = c%256;
      if (c == 27)
      {
         demoDone_ = 1;
         return 0;
      }
      else if (c == 82)
      {
         demoThresh_ += .02;
      }
      else if (c == 84)
      {
         demoThresh_ -= .02;
         if(demoThresh_ <= .02) demoThresh_ = .02;
      }
      else if (c == 83)
      {
         demoHier_ += .02;
      }
      else if (c == 81)
      {
         demoHier_ -= .02;
         if(demoHier_ <= .0) demoHier_ = .0;
      }
      return 0;
   }

   void *YoloObjectDetector::displayLoop(void *ptr)
   {
      while (1)
      {
         displayInThread(0);
      }
   }

   void *YoloObjectDetector::detectLoop(void *ptr)
   {
      while (1)
      {
         detectInThread();
      }
   }

   void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh, char **names, int classes, int delay, char *prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen)
   {
      demoPrefix_ = prefix;
      demoDelay_ = delay;
      demoFrame_ = avg_frames;
      image **alphabet = load_alphabet_with_file(datafile);
      demoNames_ = names;
      demoAlphabet_ = alphabet;
      demoClasses_ = classes;
      demoThresh_ = thresh;
      demoHier_ = hier;
      fullScreen_ = fullscreen;
      printf("YOLO V3\n");
      net_ = load_network(cfgfile, weightfile, 0);
      set_batch_network(net_, 1);
   }

   void YoloObjectDetector::yolo()
   {
      const auto wait_duration = std::chrono::milliseconds(2000);
      while (!getImageStatus())
      {
         printf("Waiting for image.\n");
         if (!isNodeRunning())
         {
            return;
         }
         std::this_thread::sleep_for(wait_duration);
      }

      std::thread detect_thread;
      std::thread fetch_thread;

      srand(2222222);

      int i;
      demoTotal_ = sizeNetwork(net_);
      predictions_ = (float **) calloc(demoFrame_, sizeof(float*));

      for (i = 0; i < demoFrame_; ++i)
      {
         predictions_[i] = (float *) calloc(demoTotal_, sizeof(float));
      }

      avg_ = (float *) calloc(demoTotal_, sizeof(float));

      layer l = net_->layers[net_->n - 1];
      roiBoxes_ = (darknet_ros::RosBox_ *) calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));

      IplImage* ROS_img = getIplImage();
      buff_[0] = ipl_to_image(ROS_img);
      buff_[1] = copy_image(buff_[0]);
      buff_[2] = copy_image(buff_[0]);
      buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
      buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
      buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
      ipl_ = cvCreateImage(cvSize(buff_[0].w, buff_[0].h), IPL_DEPTH_8U, buff_[0].c);

      int count = 0;

      if (!demoPrefix_ && viewImage_)
      {
         cvNamedWindow("YOLO V3", CV_WINDOW_NORMAL);
         if (fullScreen_)
         {
            cvSetWindowProperty("YOLO V3", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
         }
         else
         {
            cvMoveWindow("YOLO V3", 0, 0);
            cvResizeWindow("YOLO V3", 640, 480);
         }
      }

      demoTime_ = what_time_is_it_now();

      while (!demoDone_)
      {
         buffIndex_ = (buffIndex_ + 1) % 3;
         fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);
         detect_thread = std::thread(&YoloObjectDetector::detectInThread, this);
         if (!demoPrefix_)
         {
            fps_ = 1./(what_time_is_it_now() - demoTime_);
            demoTime_ = what_time_is_it_now();
            if (viewImage_)
            {
               displayInThread(0);
            }
            publishInThread();
         }
         else
         {
            char name[256];
            sprintf(name, "%s_%08d", demoPrefix_, count);
            save_image(buff_[(buffIndex_ + 1) % 3], name);
         }
         fetch_thread.join();
         detect_thread.join();
         ++count;
         if (!isNodeRunning())
         {
            demoDone_ = true;
         }
      }
   }

   IplImage* YoloObjectDetector::getIplImage()
   {
      boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
      IplImage* ROS_img = new IplImage(camImageCopy_);
      return ROS_img;
   }

   bool YoloObjectDetector::getImageStatus(void)
   {
      boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
      return imageStatus_;
   }

   bool YoloObjectDetector::isNodeRunning(void)
   {
      boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
      return isNodeRunning_;
   }

   void *YoloObjectDetector::publishInThread()
   {
      // Publish image.
      cv::Mat cvImage = cv::cvarrToMat(ipl_);
      if (!publishDetectionImage(cv::Mat(cvImage)))
      {
         ROS_DEBUG("Detection image has not been broadcasted.");
      }

      // Publish bounding boxes and detection result.
      int p = 0; //uia
      int num = roiBoxes_[0].num;
      if (num > 0 && num <= 100)
      {
         for (int i = 0; i < num; i++)
         {
            for (int j = 0; j < numClasses_; j++)
            {
               if (roiBoxes_[i].Class == j)
               {
                  rosBoxes_[j].push_back(roiBoxes_[i]);
                  rosBoxCounter_[j]++;
               }
            }
         }

         std_msgs::Int8 msg;
         msg.data = num;
         objectPublisher_.publish(msg);

         for (int i = 0; i < numClasses_; i++)
         {
            if (rosBoxCounter_[i] > 0)
            {
               darknet_ros_msgs::BoundingBox boundingBox;

               for (int j = 0; j < rosBoxCounter_[i]; j++)
               {
                  int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
                  int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
                  int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
                  int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;
      		  float Uc = ((xmax-xmin)/2+xmin); //uia: get pixel center(u) for color image
      		  float Vc = ((ymax-ymin)/2+ymin); //uia: get pixel center(u) for color image
		  float Uir = Uc; //uia: get pixel center(u) for depth image. Edit: Now the same as color image
		  float Vir = Vc; //uia: get pixel center(u) for depth image. Edit: Now the same as color image
		  float Virmax = ymax; //uia: get pixel coordinate of top of person Edit: Now the same as ymax
	          float Virmin = ymin; //uia: get pixel coordinate of top of person Edit: Now the same as ymin
                  YoloObjectDetector::Coordinates(i, xmin, ymin, xmax, ymax, Uc, Vc, Uir, Vir, Virmax, Virmin, num, p);

                  boundingBox.Class = classLabels_[i];
                  boundingBox.probability = rosBoxes_[i][j].prob;
                  boundingBox.xmin = xmin;
                  boundingBox.ymin = ymin;
                  boundingBox.xmax = xmax;
                  boundingBox.ymax = ymax;
                  boundingBox.X = X; //uia: local 3D-coordinates of detected person
                  boundingBox.Y = Y; //uia: local 3D-coordinates of detected person
                  boundingBox.Z = Z; //uia: local 3D-coordinates of detected person
                  boundingBox.Xg = Xg; //uia: global 3D-coordinates of detected person
                  boundingBox.Yg = Yg; //uia: global 3D-coordinates of detected person
                  boundingBox.Zg = Zg; //uia: global 3D-coordinates of detected person
		  boundingBox.Ymax = Ymax; //uia
		  boundingBox.Ymin = Ymin; //uia
		  boundingBox.Uc = Uc; //uia
		  boundingBox.Vc = Vc; //uia
		  boundingBox.Uir = Uir; //uia
		  boundingBox.Vir = Vir; //uia
		  boundingBox.Virmax = Virmax; //uia
		  boundingBox.Virmin = Virmin; //uia
                  boundingBox.Invalid = Invalid;
		  boundingBox.num = num; //uia
                  boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
		  p++; //uia
               }
            }
         }

         boundingBoxesResults_.header.stamp = ros::Time::now();
         boundingBoxesResults_.header.frame_id = "detection";
         boundingBoxesResults_.image_header = imageHeader_;
         boundingBoxesPublisher_.publish(boundingBoxesResults_);


         poseArray_.header.stamp = ros::Time::now();
         poseArray_.header.frame_id = "pose";
         poseArray_.poses[0].position.x = 2;
         poseArray_.poses[0].position.y = 2;
         poseArray_.poses[0].position.z = 2;
         poseArray_.poses[0].orientation.x = 0;
         poseArray_.poses[0].orientation.y = 0;
         poseArray_.poses[0].orientation.z = 0;
         poseArray_.poses[0].orientation.w = 0;
         poseArrayPublisher_.publish(poseArray_);
      }
      
      else
      {
         std_msgs::Int8 msg;
         msg.data = 0;
         objectPublisher_.publish(msg);
      }

      if (isCheckingForObjects())
      {
         ROS_DEBUG("[YoloObjectDetector] check for objects in image.");
         darknet_ros_msgs::CheckForObjectsResult objectsActionResult;
         objectsActionResult.id = buffId_[0];
         objectsActionResult.bounding_boxes = boundingBoxesResults_;
         checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes.");
      }

      boundingBoxesResults_.bounding_boxes.clear();
      for (int i = 0; i < numClasses_; i++)
      {
         rosBoxes_[i].clear();
         rosBoxCounter_[i] = 0;
      }

      return 0;
   }

   void YoloObjectDetector::Coordinates(int ObjID, int xmin, int ymin, int xmax, int ymax, float Uc, float Vc, float Uir, float Vir, float Virmax, float Virmin, int num, int p)
   {
	//uia start :::::::
      
	
      int Ind=0;

      float Value=0;
 	// find how many valid depth measurements in a 7x7 region around the center of the detected person-center
	  for(int i=(Uir-3); i<=(Uir+3); i++)
         for(int j=(Vir-3); j<=(Vir+3); j++)
         {
            Value=(float)DepthImageCopy_.at<float>(j,i);
            if (Value>1000 && Value<9000)
            {	
		Ind++;
            }
         }

      float depthArray[Ind]; // Initialize an array of size corresponding to the valid depth-measurments
	Ind = 0;

	// fill the array with the depth-values
	  for(int i=(Uir-3); i<=(Uir+3); i++)
         for(int j=(Vir-3); j<=(Vir+3); j++)
         {
            Value=(float)DepthImageCopy_.at<float>(j,i);
            if (Value>1000 && Value<9000)
            {	
		Ind++;
		depthArray[Ind] = Value;
            }
         }	
	// organize the array from smallest to largest values
	for(int i=0; i<Ind; i++)
	{
		for(int j=i+1; j<Ind; j++)
		{
			if(depthArray[j] < depthArray[i])
			{
			int temp = depthArray[i];
			depthArray[i] = depthArray[j];
			depthArray[j] = temp;
			}
		}
	}
	
		
	double Z_ = 0;
	// find the median
	if(Ind%2 == 0)
	{
		Z_ = (depthArray[Ind/2] + depthArray[Ind/2 - 1]) / 2.0;	
	}
		
	else
	{
      		Z_ = depthArray[Ind/2];
	}

      Invalid= true;
      if (Value > 0)
      	{
	 double Cy;
	 double fy;
	 double Cx;
	 double fx;
         // get the camera coefficients
	 nodeHandle_.param("Cx", Cx, 252.0);
         nodeHandle_.param("fx", fx, 365.0);
	 nodeHandle_.param("Cy", Cy, 207.0);
         nodeHandle_.param("fy", fy, 365.0);

         Invalid= false;
       	 // Calculate the local 3D-coordinates
       	 double X_=((Uir-Cx)*Z_)/fx;
       	 double Y_=((Vir-Cy)*Z_)/fy;
	 double Ymax=((Virmax-Cy)*Z_)/fy;
	 double Ymin=((Virmin-Cy)*Z_)/fy;
	 
	 // add a bit of length since the 3D-coordinates correspond to the face of the persons body
	 double lengthAdded;
	 nodeHandle_.param("lengthAdded", lengthAdded, 200.0);
	 double L = sqrt(X_ * X_ + Y_ * Y_ + Z_ * Z_);
	 double a = (L + lengthAdded)/L;
	 
	 // Update the coordinates
	 X = X_ * a;
	 Y = Y_ * a;
 	 Z = Z_ * a;



         double xr;
         double yr;
         double zr;
         double xt;
         double yt;
         double zt;

	 // get the translation and rotation of the camera relative to the global coordinate-system
         nodeHandle_.getParam("xr", xr);
         nodeHandle_.getParam("yr", yr);
         nodeHandle_.getParam("zr", zr);
         nodeHandle_.getParam("xt", xt);
         nodeHandle_.getParam("yt", yt);
         nodeHandle_.getParam("zt", zt);

         double localPoint[3][1] = {X, Y, Z};
	 //rotation matrices
         double Rx[3][3] = {{1, 0, 0}, {0, cos(xr), -sin(xr)}, {0, sin(xr), cos(xr)}};
         double Ry[3][3] = {{cos(yr), 0, sin(yr)}, {0, 1, 0}, {-sin(yr), 0, cos(yr)}};
         double Rz[3][3] = {{cos(zr), -sin(zr), 0}, {sin(zr), cos(zr), 0}, {0, 0, 1}};
         
         double Rzy[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
         double R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
         double RlP[3][1] = {0, 0, 0};

        
	 // calculate
         for(int i = 0; i < 3; i++)
             for(int j = 0; j < 3; j++)
             {
                 for(int k = 0; k < 3; k++)
                 Rzy[i][j] += Rz[i][k] * Ry[k][j];
             }

         for(int i = 0; i < 3; i++)
             for(int j = 0; j < 3; j++)
             {
                 for(int k = 0; k < 3; k++)
                 R[i][j] += Rzy[i][k] * Rx[k][j];
             }

         for(int i = 0; i < 3; i++)
         {
             for(int j = 0; j < 3; j++)
                 RlP[i][0] += R[i][j] * localPoint[j][0];
         }


	 // get the global 3D-coordinates
         Xg = xt + RlP[0][0];
         Yg = yt + RlP[1][0];
         Zg = zt + RlP[2][0];
	


         float Xg_dummy = Xg;
         float Yg_dummy = Yg;
	 

	 // ignore false detections
         if(Zg < 400 || Zg > 2000)
        {
            Xg = 0.0;
            Yg = 0.0;
            Zg = 0.0;
        }

         if(abs(Xg-Xg_temp[p]) > 400 || abs(Yg-Yg_temp[p]) > 400)
         {
             Xg = 0.0;
             Yg = 0.0;
             Zg = 0.0;
         }

         Xg_temp[p] = Xg_dummy;
         Yg_temp[p] = Yg_dummy;

	 //uia end :::::::
      	}
      }
   }

