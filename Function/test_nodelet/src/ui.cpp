#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <test_nodelet/BoardInfo.h>
#include <test_nodelet/Calibrate.h>
using namespace cv;
using namespace std;
bool select_flag = false;

int flag = 0;

Mat frame;

int center_x = 375;//圆心横坐标
int center_y_chessbore = 250;//圆心1纵坐标
int center_y_calibrate = 330;
int center_y_hough = 410;
int r = 20;//圆半径


class UI
{
public:

    void showImage(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("<<<<<<<<<<<<imageCallback ");
        try
        {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat image( frame.rows, frame.cols + 100, CV_8UC3, cv::Scalar::all(0));


            center_x = frame.cols + 50;
            center_y_chessbore = frame.rows/3;
            center_y_calibrate = center_y_chessbore + 100;
            center_y_hough = center_y_calibrate + 100;

            Point center1 = Point(center_x,center_y_chessbore);
            Point center2 = Point(center_x,center_y_calibrate);
            Point center3= Point(center_x,center_y_hough);

            Point textLoc1 = Point(frame.cols + 20,center_y_chessbore+40);
            Point textLoc2 = Point(frame.cols + 10,center_y_calibrate+40);
            Point textLoc3= Point(frame.cols + 20,center_y_hough+40);

            Scalar disableColor = Scalar(96,96,96);
            Scalar enableColor = Scalar(255,255,0);
            Scalar white = Scalar(255,255,255);
            if(flag== 0){
                circle(image, center1, r, enableColor, -1);
                circle(image, center2, r, disableColor, -1);
                circle(image, center3, r, disableColor, -1);

            }else if(flag == 1){
                circle(image, center1, r, disableColor, -1);
                circle(image, center2, r, enableColor, -1);
                circle(image, center3, r, disableColor, -1);
            }else{
                circle(image, center1, r, disableColor, -1);
                circle(image, center2, r, disableColor, -1);
                circle(image, center3, r, enableColor, -1);
            }
            putText(image, "Chess", textLoc1, 0, 0.6, white, 1, 1, false);
            putText(image, "Calibrate", textLoc2,  0,0.6, white, 1, 1, false);
            putText(image, "Hough", textLoc3, 0,0.6, white, 1, 1, false);


            cv::Rect r(0, 0, frame.cols, frame.rows);
            cv::Mat roi = image(r);
            cv::addWeighted(roi, 1.0, frame, 1.0, 0,roi);

            imshow("display", image);
            waitKey(30);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      //  if(status_ == 0 )
       // {
           // ROS_INFO("imageCallback>>>>>>>>>>>>>>>>>>>>>>");
            showImage(msg);
      //  }

    }

    void boardImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      //  if(status_ == 1)
       // {
             ROS_INFO("boardImageCallback>>>>>>>>>>>>>>>>>>>>>>");
             showImage(msg);
      //  }

    }


    void rejectCircleImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
       // if(status_ == 3)
       // {
             ROS_INFO("rejectCircleImageCallback>>>>>>>>>>>>>>>>>>>>>>");
             showImage(msg);
       // }

    }


    bool boardInfoCallback(test_nodelet::BoardInfo ::Request  &req,   test_nodelet::BoardInfo ::Response &res)
    {
        if(req.ready)
        {
            ROS_INFO("We can calibrate................................");
            //set UI button
            flag = 1;
            //waitfor calibrate button
            res.isCalibrate = true;
            imageSub_ = boost::make_shared< image_transport::Subscriber>(it_->subscribe("image", 1, &UI::imageCallback, this));
        }
        return true;
    }

    void uiCalibrate()
    {

        calibrateClient_  =  nh_->serviceClient<test_nodelet::Calibrate>("calibrates");
        test_nodelet::Calibrate  srv;
        srv.request.ready = true;
        ROS_INFO("scall>>>>>>>>>>>>>>>>>>>>>>>>>");
        if(calibrateClient_.call(srv))
        {
            if(srv.response.isHough)
            {
                flag = 2;
            }
        }
    }

    void   checkBoard()
    {

        boardImageSub_ = boost::make_shared< image_transport::Subscriber>(it_->subscribe("boardImage", 1, &UI::boardImageCallback, this));

        boardInfoService_ = nh_->advertiseService("boardInfo", &UI::boardInfoCallback, this);

    }

    void hough()
    {

         rejectCircleImageSub_ =  boost::make_shared< image_transport::Subscriber>(it_->subscribe("rejectCircleImage", 1, &UI::rejectCircleImageCallback, this));
    }

    virtual void onInit()
    {
        ROS_INFO(">>>>>onInit()>>>>>>>>>>>>>>>>>>\n");
        nh_ = boost::make_shared<ros::NodeHandle>();
        it_  = boost::make_shared<image_transport::ImageTransport>(*nh_);
        imageSub_ = boost::make_shared< image_transport::Subscriber>(it_->subscribe("image", 1, &UI::imageCallback, this));
    }
  public:
    boost::shared_ptr<ros::NodeHandle> nh_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr< image_transport::Subscriber>       imageSub_;
    boost::shared_ptr< image_transport::Subscriber>       boardImageSub_;
    boost::shared_ptr< image_transport::Subscriber>       rejectCircleImageSub_;
    ros::ServiceServer boardInfoService_;
    ros::ServiceClient calibrateClient_;
};


void onMouse(int event,int x,int y,int flags,void*param)
{
    UI *ui = (UI*)param;

    if(event == EVENT_LBUTTONDOWN && x>(center_x-r) && x<(center_x+r))
    {


        if((center_y_chessbore-r) <= y && y< (center_y_chessbore+r))
        {


                //开始获取棋盘格数据
                ui->imageSub_.reset();
                ui->checkBoard();
                flag = 0;

        }
        if((center_y_calibrate-r) <= y && y< (center_y_calibrate+r))
        {

                //第二个按钮可点击，以下是点击后要实现的逻辑：开始标定，获得标定参数。应该获得后期给发布的消息后再flag=2,这里赋值是为了测试。
                ROS_INFO(">>>>>>>#################click flag ==1");
                ui->boardImageSub_.reset();
                ui->uiCalibrate();

        }
        if((center_y_hough-r) <= y && y< (center_y_hough+r))
        {

                ui->imageSub_.reset();
                ui->hough();
         }

    }
}






int main(int argc, char** argv)
{
    ros::init(argc, argv, "ui");
    namedWindow("display");
    UI *ui = new UI();
    ui->onInit();
    setMouseCallback("display",onMouse, ui);
    while(1)
    {
        ros::spinOnce();
     }
    delete ui;
    return 0;

}
