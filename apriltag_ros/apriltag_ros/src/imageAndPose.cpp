#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
//画坐标轴
void DrawFrameAxes(InputOutputArray image, InputArray cameraMatrix, InputArray distCoeffs,
                   InputArray rvec, InputArray tvec, float length, int thickness)
{

    CV_Assert(image.getMat().total() > 0);
    CV_Assert(length > 0);

    // project axes points
    std::vector<Point3f> axesPoints;
    axesPoints.push_back(Point3f(0, 0, 0));
    axesPoints.push_back(Point3f(length, 0, 0));
    axesPoints.push_back(Point3f(0, length, 0));
    axesPoints.push_back(Point3f(0, 0, length));
    std::vector<Point2f> imagePoints;
    projectPoints(axesPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    // draw axes lines
    line(image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), thickness);
    line(image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), thickness);
    line(image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), thickness);
}
//
//

//
void imageCallback(const sensor_msgs::ImageConstPtr &msg, const apriltag_ros::AprilTagDetectionArray::ConstPtr &data)
{
    try
    {
        if (data->detections.size() > 0)
        {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat imageCopy;
            double fx = 916.220458984375;
            double cx = 648.5140380859375;
            double fy = 914.3749389648438;
            double cy = 379.0440368652344;
            double k1 = 0;
            double k2 = 0.;
            double p1 = 0;
            double p2 = 0;
            double k3 = 0;
            cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << fx, 0.0, cx,
                                    0.0, fy, cy,
                                    0.0, 0.0, 1.0);
            cv::Mat distCoeffs = (cv::Mat_<float>(5, 1) << 0, k2, p1, p2, k3);
            image.copyTo(imageCopy);
            cv::Vec3d rvec, tvec;
            Eigen::Quaterniond quater(data->detections[0].pose.pose.pose.orientation.w, data->detections[0].pose.pose.pose.orientation.x, data->detections[0].pose.pose.pose.orientation.y, data->detections[0].pose.pose.pose.orientation.z);
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix = quater.matrix();
            cv::Mat R;
            cv::eigen2cv(rotation_matrix, R);
            cv::Rodrigues(R, rvec);

            tvec[0] = data->detections[0].pose.pose.pose.position.x;
            tvec[1] = data->detections[0].pose.pose.pose.position.y;
            tvec[2] = data->detections[0].pose.pose.pose.position.z;
            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
            // DrawFrameAxes(imageCopy,cameraMatrix,distCoeffs,rvec,tvec,1,1);
            std::cout << "POSE and Orientation" << std::endl;
            std::cout << "POSE:X  " << data->detections[0].pose.pose.pose.position.x << std::endl;
            std::cout << "POSE:Y  " << data->detections[0].pose.pose.pose.position.y << std::endl;
            std::cout << "POSE:Z  " << data->detections[0].pose.pose.pose.position.z << std::endl;
            std::cout << "Orientation:X  " << data->detections[0].pose.pose.pose.orientation.x << std::endl;
            std::cout << "Orientation:Y  " << data->detections[0].pose.pose.pose.orientation.y << std::endl;
            std::cout << "Orientation:Z  "  << data->detections[0].pose.pose.pose.orientation.z << std::endl;
            std::cout << "Orientation:W  " << data->detections[0].pose.pose.pose.orientation.w << std::endl;
            cv::imshow("view", imageCopy);
        }
        else
        {
            std::cout << "no find" << std::endl;
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::imshow("view", image);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    int a;
    ros::NodeHandle nh;
    ros::Rate r(10);
    cv::namedWindow("view");
    cv::startWindowThread();
    message_filters::Subscriber<Image> image_sub(nh, "tag_detections_image", 1);
    message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> info_sub(nh, "tag_detections", 1);
    TimeSynchronizer<Image, apriltag_ros::AprilTagDetectionArray> sync(image_sub, info_sub, 1);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));
    ros::spin();

    return 0;
}
