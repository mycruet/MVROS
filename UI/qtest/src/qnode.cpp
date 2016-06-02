#include "../include/qtest/qnode.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

QNode::QNode(int argc, char** argv) :	init_argc(argc),	init_argv(argv)
{



}


QNode::~QNode()
{

}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

     ROS_INFO(">>>>imageCallback>>>>>>>>>>>>>>>>>>>");
     cv::Mat frameBGR;//bgr
     cv::Mat frameRGB;//rgb
     QImage qtImage;
//ROS process
     try
     {
                     frameBGR = cv_bridge::toCvShare(msg, "bgr8")->image;
     }
     catch (cv_bridge::Exception& e)
     {
                 ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
//Qt process

     if(frameBGR.channels() == 3)
     {
         //cv_brige.so must be use opencv 3.0
         cv::cvtColor(frameBGR, frameRGB, CV_BGR2RGB);

         qtImage  = QImage((const uchar *)frameRGB.data, frameRGB.cols, frameRGB.rows,  frameRGB.cols*frameRGB.channels(), QImage::Format_RGB888);
     }
     else
     {

        qtImage  = QImage((const uchar *)frameBGR.data, frameBGR.cols, frameBGR.rows,  frameBGR.cols*frameBGR.channels(), QImage::Format_RGB888);

     }

     Q_EMIT imageShow(qtImage);


}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"ui");
    ROS_INFO(">>>>>onInit()>>>>>>>>>>>>>>>>>>\n");
    nh_ = boost::make_shared<ros::NodeHandle>();
    it_  = boost::make_shared<image_transport::ImageTransport>(*nh_);
    imageSub_ = boost::make_shared<image_transport::Subscriber>(it_->subscribe("image", 1, &QNode::imageCallback, this));
    //start thread
    start();
    return true;
}

void QNode::run()
{
    while(ros::ok())
    {

        ros::spinOnce();

    }

}


bool QNode::boardInfoCallback(test_nodelet::BoardInfo ::Request  &req,   test_nodelet::BoardInfo ::Response &res)
    {
        if(req.ready)
        {
            ROS_INFO("We can calibrate................................");
            res.isCalibrate = true;
            //boardImageSub_.reset();
            //imageSub_ = boost::make_shared< image_transport::Subscriber>(it_->subscribe("image", 1, &QNode::imageCallback, this));
             Q_EMIT canCalibrate();
        }
        return true;
    }

void   QNode::checkBoard()
{
    ROS_INFO("checkBoard()................................");
    imageSub_.reset();
    boardImageSub_ = boost::make_shared< image_transport::Subscriber>(it_->subscribe("boardImage", 1, &QNode::imageCallback, this));
    boardInfoService_ = nh_->advertiseService("boardInfo", &QNode::boardInfoCallback, this);

}

void QNode::houghCircle()
{
    boardImageSub_.reset();
    rejectCircleImageSub_ =  boost::make_shared< image_transport::Subscriber>(it_->subscribe("rejectCircleImage", 1, &QNode::imageCallback, this));
}

bool QNode::calibrate()
{
    calibrateClient_  =  nh_->serviceClient<test_nodelet::Calibrate>("calibrates");
    test_nodelet::Calibrate  srv;
    srv.request.ready = true;
    ROS_INFO("scall>>>>>>>>>>>>>>>>>>>>>>>>>");
    if(calibrateClient_.call(srv))
    {
        if(srv.response.isHough)
        {
            return true;
        }
    }
    return false;
}











