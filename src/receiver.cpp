#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void ColorCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv::imshow("D435/color", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void DepthCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv::imshow("D435/depth", cv_bridge::toCvShare(msg, "mono16")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle image_listener;
    cv::namedWindow("D435/color");
    cv::namedWindow("D435/depth");
    image_transport::ImageTransport it(image_listener);
    image_transport::Subscriber sub_color = it.subscribe("D435/color", 1, ColorCallback);
    image_transport::Subscriber sub_depth = it.subscribe("D435/depth", 1, DepthCallback);
    ros::Rate rate(30.0);
    while (image_listener.ok()) {
        ros::spinOnce(); // ros::spin执行所有subscriber和advertiser
        rate.sleep();
    }
}