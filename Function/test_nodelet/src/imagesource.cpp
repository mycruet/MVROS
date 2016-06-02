#ifndef IMAGESOURCE_CPP
#define IMAGESOURCE_CPP

#endif // IMAGESOURCE_CPP
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <test_nodelet/BoardInfo.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {
    ROS_INFO("imageCallback......................");
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
 }

bool boardCallback(test_nodelet::BoardInfo ::Request  &req,   test_nodelet::BoardInfo ::Response &res)
{
    if(req.ready)
    {
        ROS_INFO("We can calibrate................................");
        //set UI button
        res.isCalibrate = true;
    }
    return true;
}


 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "image_publisher");
     ros::NodeHandle nh;
     image_transport::ImageTransport it(nh);
     image_transport::Publisher  pub = it.advertise("image", 1);
   //image_transport::Subscriber hough_sub = it.subscribe("hough_circles/images", 1, imageCallback);
  // image_transport::Subscriber board_sub =  it.subscribe("boardImage", 1, imageCallback);
   //ros::ServiceServer service = nh.advertiseService("boardInfo", boardCallback);

#if 1
     cv::Mat frame  = cv::imread("/home/frank/testopenapp/circle.jpg", 1);
     //cv::Mat frame  = cv::imread("test0.jpg", 1);
     if(frame.empty())
     {
         ROS_INFO("read image error\n");

     }
     //cv::imshow("view", frame);
     cv::resize(frame,frame,cv::Size(410,372),0,0,CV_INTER_LINEAR);  
     cv::waitKey(30);
     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();


     ros::Rate loop_rate(50);
     while (nh.ok())
     {
         pub.publish(msg);

         ros::spinOnce();
         loop_rate.sleep();
     }


#endif

#if 0
     cv::VideoCapture cap(0);
     // Check if video device can be opened with the given index
     if(!cap.isOpened()) return 1;
     cv::Mat frame;
     sensor_msgs::ImagePtr msg;


     ros::Rate loop_rate(1500);
     while (nh.ok())
     {
         cap >> frame;
         // Check if grabbed frame is actually full with some content
         if(!frame.empty())
         {
             cv::imshow("view", frame);
             msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
             pub.publish(msg);
             cv::waitKey(1);
         }

         ros::spinOnce();
         loop_rate.sleep();
     }
#endif
  }





