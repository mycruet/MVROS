
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <test_nodelet/CameraCalibrator.h>
#include <test_nodelet/Setting.h>
#include <test_nodelet/BoardInfo.h>
#include <test_nodelet/Calibrate.h>


//configure setting
   //Setting set(9, 6, 0, 0,Setting::Opencv3_Chessboard_File);
   Setting set(9, 6, 0, 0,Setting::Opencv3_Chessboard_Camera);

   //selected board size from setting
//cv::Size boardSize(set.bszie_x_, set.bszie_y_);
 bool runable = true;

class monocalibrate
{

public:
    monocalibrate(){

    }

    virtual ~monocalibrate(){

    }


    void checkBoardCallback(const sensor_msgs::ImageConstPtr& msg)
    {

        ROS_INFO("imageCallback.....................");

        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }




        if(cam_->addChessboardPoints(frame) == true)
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub_->publish(msg);
            //cv::imshow("test", frame);
           // cv::waitKey(30);
#if  1
            if(cam_->checkPrepareForCalibrate() == true)
            {
                ROS_INFO("prepare calibrate ok,     stop checkboard............");

                test_nodelet::BoardInfo srv;
                srv.request.ready = true;
                if (client_.call(srv))
                {
                    if(srv.response.isCalibrate)
                    {
                       //this may clear all msg sended before in the msg buffer but not reach to client
                            sub_.reset();
                    }
                }
                else
                {
                    ROS_ERROR("Failed to call service  boardInfo");
		    //runable = false;
                 }
            }
    #endif
        }
        else
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub_->publish(msg);
            ROS_INFO("send2............");
            //cv::imshow("test", frame);
            //cv::waitKey(30);
        }

    }

    bool calibrateCallback(test_nodelet::Calibrate::Request  &req,   test_nodelet::Calibrate::Response &res)
    {
         ROS_INFO(" calibrate................................");
        if(req.ready)
        {

            if(cam_->calibrate())
            {
                    res.isHough = true;
                    runable = false;
                    ROS_INFO("runable  >>>>>>2%d", runable);
            }
        }
        return true;
    }


    virtual void onInit()
    {
        ROS_INFO(">>>>>onInit()>>>>>>>>>>>>>>>>>>\n");
        cam_ =  boost::make_shared<CameraCalibrator>(cv::Size(set.bszie_x_, set.bszie_y_));
        nh_ = boost::make_shared<ros::NodeHandle>();
        it_  = boost::make_shared<image_transport::ImageTransport>(*nh_);
        sub_ =  boost::make_shared<image_transport::Subscriber>(it_->subscribe("image", 1, &monocalibrate::checkBoardCallback, this));
        pub_ =  boost::make_shared<image_transport::Publisher>(it_->advertise("boardImage", 1));
        client_ = nh_->serviceClient<test_nodelet::BoardInfo>("boardInfo");
        cali_    = nh_->advertiseService("calibrates", &monocalibrate::calibrateCallback, this);

    }


    boost::shared_ptr<ros::NodeHandle> nh_;
    boost::shared_ptr<image_transport::ImageTransport>  it_;
    boost::shared_ptr<image_transport::Subscriber>          sub_;
    boost::shared_ptr<image_transport::Publisher>            pub_;

    boost::shared_ptr<CameraCalibrator> cam_;
    ros::ServiceClient client_;
    ros::ServiceServer cali_;
};


 int main(int argc, char **argv)
  {

     ros::init(argc, argv, "monoCalibrate");

     monocalibrate ca;
     ca.onInit();
     while(runable)
     {
         //ROS_INFO("runable  >>>>>>%d", runable);
         ros::spinOnce();
      }

  }
