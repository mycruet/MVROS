
#include <ros/ros.h>
 #include <image_transport/image_transport.h>
  #include <opencv2/highgui/highgui.hpp>
  #include <cv_bridge/cv_bridge.h>

 int main(int argc, char** argv)
 {
    ros::init(argc, argv, "image_publisher");
     ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image", 1);
     cv::Mat image = cv::imread("circle.jpg", 1);
    cv::waitKey(30);
    if(image.empty())
    {
         ROS_INFO("read image error\n");

    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  #if 1
    int i = 0;
   ros::Rate loop_rate(2000);
   while (nh.ok()  && i < 1000) {
   pub.publish(msg);
   i++;
   ROS_INFO(">>>>%d", i);
    //ros::spinOnce();
    loop_rate.sleep();
   }
   ros::spin();
#endif
  }
