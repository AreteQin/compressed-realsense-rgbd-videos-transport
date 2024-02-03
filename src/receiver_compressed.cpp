#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void ColorCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv::imshow("/camera/color/image_raw", cv_bridge::toCvShare(msg, "rgb8")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
    // calculate time delay
    auto now = ros::Time::now();
    ROS_INFO("time delay: %f", now.toSec() - (msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9));
}

void DepthCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv::imshow("/camera/depth/image_rect_raw", cv_bridge::toCvShare(msg, "mono16")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle image_listener;
    image_listener.setParam("image_transport", "compressed");
    cv::namedWindow("/camera/color/image_raw");
    cv::namedWindow("/camera/depth/image_rect_raw");
    image_transport::ImageTransport it(image_listener);
    image_transport::Subscriber sub_color = it.subscribe("/camera/color/image_raw", 1, ColorCallback);
    image_transport::Subscriber sub_depth = it.subscribe("/camera/depth/image_rect_raw", 1, DepthCallback);
    ros::Rate rate(30.0);
    while (image_listener.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}