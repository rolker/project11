# Replay and extract images from a bag file.

## Prerequisits

- A bag file with recorded images
- Image view and transport packages

    sudo apt install ros-melodic-image-view
    sudo apt install ros-melodic-image-transport-plugins
    
## Viewing images

Adapt the following commands to actual topics. Note that compressed topics, such as /camera_1/image_raw/compressed get specified as /camera_1/image_raw with _image_transport:=compressed as an argument.

Open a terminal and start a ROS core.

Replay a bag file, in a new terminal:

    rosbag play camera_1.bag

Display replayed file, in a new terminal:

    rosrun image_view image_view image:=/camera_1/image_raw
    
If the image data is compressed, add the compressed argument:
    
    rosrun image_view image_view image:=/camera_1/image_raw compressed

## Extracting images

For a compressed stream:

    rosrun image_view extract_images image:=/camera_1/image_raw/  _image_transport:=compressed
    
For raw:

    rosrun image_view extract_images image:=/camera_1/image_raw/
    
