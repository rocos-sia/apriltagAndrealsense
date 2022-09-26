#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

class image_receive
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh1;
    

public:
    int num ;
    image_receive(int NUM)
    {
        image_receive::num=NUM;
    }
    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {

            std::cout<<"strat"<<std::endl;
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::imshow("view", image);
           
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
        cv::waitKey(1);
    }
    void imageReceive()
    {
        ros::Rate r(10);
        cv::namedWindow("view");
        cv::startWindowThread();
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("/tag_detections_image", 1, &image_receive::imageCallback,this);
        // ros::Subscriber sub_data = nh1.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, &image_receive::imageCallback,this);
        while (ros::ok())
        {
            num++;
            ros::spinOnce();
            std::cout<<"1"<<std::endl;
            r.sleep();
            if (num>1000)
            {
                break;
            }
        }
        
        cv::destroyWindow("view");
    }
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    image_receive image(0);
    int a;
    image.imageReceive();
    
    
    return 0;
    
}
