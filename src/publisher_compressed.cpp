#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API

int main(int argc, char **argv) {
    ros::init(argc, argv, "pub_cam_node");
    ros::NodeHandle nh;

    // Declare the RealSense pipeline, encapsulating the actual device and sensors
    std::unique_ptr<rs2::pipeline> pipe;
    rs2::config cfg;
    pipe.reset(new rs2::pipeline());
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    // Start streaming with the default recommended configuration
    rs2::pipeline_profile profile = pipe->start(cfg);

    // set depth image format parameter "D435.depth.format" to "png"
    nh.setParam("/camera/depth/image_rect_raw/compressed/format", "png");
    // image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_color = it.advertise("/camera/color/image_raw", 1);
    image_transport::Publisher pub_depth = it.advertise("/camera/depth/image_rect_raw", 1);

    cv::Mat color_cv, depth_cv;

    while (ros::ok()) {
        rs2::frameset data = pipe->wait_for_frames(); // Wait for next set of frames from the camera (must be called in a loop)

        rs2::frame color_frame = data.get_color_frame();
        rs2::frame depth_frame = data.get_depth_frame();

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        color_cv = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *) color_frame.get_data(), cv::Mat::AUTO_STEP);
        depth_cv = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);

        pub_color.publish(cv_bridge::CvImage(header, "rgb8", color_cv).toImageMsg());
        pub_depth.publish(cv_bridge::CvImage(header, "mono16", depth_cv).toImageMsg());

        ros::spinOnce();
    }
}