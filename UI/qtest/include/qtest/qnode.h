#ifndef QNODE_H
#define QNODE_H

#include <QThread>
#include <QImage>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <test_nodelet/BoardInfo.h>
#include <test_nodelet/Calibrate.h>


class QNode : public QThread
{
    Q_OBJECT
public:
    explicit QNode(int argc, char **argv);
    virtual ~QNode();
    virtual bool init();
    //callback from other rosnode
     void  imageCallback(const sensor_msgs::ImageConstPtr& msg);
     bool  boardInfoCallback(test_nodelet::BoardInfo ::Request  &req,   test_nodelet::BoardInfo ::Response &res);
     void  checkBoard();
     bool  calibrate();
     void  houghCircle();


 protected:
      void run();
Q_SIGNALS:
      void imageShow(QImage q);
      void canCalibrate();



 private:
    int init_argc;
    char** init_argv;

    boost::shared_ptr<ros::NodeHandle> nh_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<image_transport::Subscriber>       imageSub_;
    boost::shared_ptr<image_transport::Subscriber>       boardImageSub_;
    boost::shared_ptr<image_transport::Subscriber>       rejectCircleImageSub_;
    ros::ServiceServer boardInfoService_;
    ros::ServiceClient calibrateClient_;
};

#endif // QNODE_H
