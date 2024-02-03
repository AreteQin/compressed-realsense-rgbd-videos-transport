# compressed-realsense-rgbd-videos-transport
Transport compressed Realsense RGBD videos using ROS image_transport to reduce latency.

## Do the following three steps on both publisher and receiver

### Install dependencies
#### OpenCV
```bash
sudo apt install libopencv-dev
```
#### ROS
https://www.ros.org/
#### Realsense SDK
```bash
sudo apt install ros-${ROS_DISTRO}-realsense2-camera
```

### Create ROS workspace and download this package
```
mkdir -p ~/video_transport/src
cd video_transport/src
catkin_init_workspace 
cd ..
catkin_make
cd src
git clone https://github.com/AreteQin/compressed-realsense-rgbd-videos-transport.git
```

### Make
```
cd ..
catkin build
```
### Run the launch file
```
rosrun compressed_realsense_rgbd_videos_transport D435_receiver_compressed _image_transport:=compressed
rosrun compressed_realsense_rgbd_videos_transport D435_receiver_compressed _image_transport:=raw
```

### Change the depth image encoding
```
rosrun dynamic_reconfigure dynparam set /camera/depth/image_rect_raw/compressed format png
```
