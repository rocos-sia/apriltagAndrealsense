#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <iostream>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <math.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
using namespace ur_rtde;
using namespace cv;
using namespace Eigen;

class Apriltag_receive
{
private:
    int sum = 200; //类加次数
    double dist = 0.3;
    cv::Vec3d tvec_sum;
    int count = 0;

    Mat R_1;
    Matrix<double, 3, 3> R_matrix;
    double fx = 1364.61;
    double cx = 972;
    double fy = 1361.76;
    double cy = 556.603;

    double k1 = 0;
    double k2 = 0.;
    double p1 = 0;
    double p2 = 0;
    double k3 = 0;

public:
    void callback_apriltag(const apriltag_ros::AprilTagDetectionArray::ConstPtr &data)
    {
        if (data->detections.size() > 0)
        {
            std::cout << (data->detections[0].pose.pose.pose.position.x) << std::endl;
            ROS_INFO("x", data->detections[0].pose.pose.pose.position.x);
        }
        else
        {
            std::cout << "apriltag识别错误" << std::endl;
        }
    }
    void apriltag_init()
    {
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, &Apriltag_receive::callback_apriltag,this);

    }
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "listener_apriltag");

    

    return 0;
}