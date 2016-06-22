#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/opencv.hpp> 

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>  
#include <iomanip>  
#include <fstream>  
#include "std_msgs/String.h" 
  
using namespace std;  
bool hasHough = false;

namespace test_nodelet
{

class hough_circles_nodelet : public nodelet::Nodelet
{


    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO(">>>>>imageCallback>>>>>>>>>>>>>>>>>>\n");
        //cv::namedWindow( "hello", cv::WINDOW_AUTOSIZE );

        // Convert the image into something opencv can handle.
        //已经检测到霍夫圆后 就停止发送图像
        if(hasHough){
            return;
        }
        cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;

        if(frame.empty())
        {
            ROS_INFO("Convert the image error\n");

        }

        // Do the work
        std::vector<cv::Rect> faces;
        cv::Mat src_gray, edges;

        if ( frame.channels() > 1 )
        {
            cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY );
        }
        else
        {
            src_gray = frame;
        }

        // Reduce the noise so we avoid false circle detection
        cv::GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );


        //runs the detection, and update the display
        // will hold the results of the detection
        std::vector<cv::Vec3f> circles;
        // runs the actual detection
        //cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, canny_threshold_, accumulator_threshold_, 0, 0 );
        cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 50, 100, 0, 0 );
        // clone the colour, input image for displaying purposes
	ROS_INFO("have circels .....%d\n",circles.size());
       
        for( size_t i = 0; i < circles.size(); i++ )
        {
            hasHough = true;
            ROS_INFO("have circels .....\n");
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            cv::circle( frame, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            cv::circle( frame, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );

        
            ofstream outfile;  
            outfile.open("/home/gxf/test_node/src/test_nodelet/tmp/mycircle.txt"); //myfile.bat是存放数据的文件名  
           if(outfile.is_open())  
           {  
            //  outfile<<i<<endl;
               outfile<<radius<<endl;
               outfile<<center<<endl;
               outfile.close();
               cout<<"能打开文件!"<<endl;    
           }  
           else  
          {  
              cout<<"不能打开文件!"<<endl;  
          }  
            

        }
        if(hasHough){
          std_msgs::String str_msg;
          str_msg.data = "good";
          hasHoughPub_.publish(str_msg);
        }

        // Publish the image.
        sensor_msgs::ImagePtr out_img = cv_bridge::CvImage(msg->header, msg->encoding,frame).toImageMsg();
        pub_.publish(out_img);
       // ROS_INFO("publish the image .....\n");
        

    }


    virtual void onInit()
    {
        ROS_INFO(">>>>>onInit()>>>>>>>>>>>>>>>>>>\n");
        nh_.reset(new ros::NodeHandle(getNodeHandle()));
        image_transport::ImageTransport it(*nh_);
        sub_ = it.subscribe("image", 1,  &hough_circles_nodelet::imageCallback, this);
        //pub_ = it.advertise("hough_circles/images", 1);
        pub_ = it.advertise("rejectCircleImage", 1);
        hasHoughPub_ = nh_->advertise<std_msgs::String>("hasHough", 1000);  
    }

    boost::shared_ptr<ros::NodeHandle> nh_;
    image_transport::Subscriber        sub_;
    image_transport::Publisher          pub_;
    ros::Publisher          hasHoughPub_;




};

}

PLUGINLIB_EXPORT_CLASS(test_nodelet::hough_circles_nodelet, nodelet::Nodelet) 
