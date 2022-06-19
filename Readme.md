# ROSBag Playback Controller

## Description
Small python node which allows to control the playback of rosbags. Since ROS2 Humble Hawksbill most of the features
are directly integrated into rosbag2. However, rosbag2 still misses one nice feature: a play next message 
functionality which is sensitive to certain topics (which this node has).

# Usage

Single topic:  
`ros2 run rosbag_play_controller rosbag_controller --ros-args -p topics:="[/camera/image_raw]"`

If the key N is pressed during paused playback, the rosbag will publish all messages up to the 
next message on the "/camera/image_raw" topic.

Multiple topics:  
`ros2 run rosbag_play_controller rosbag_controller --ros-args -p topics:="[/camera/image_raw, /camera/camera_info]"`

As above the playback continues until a message on either of the two specified topics is published.

Note: in both cases the playback will happen one message after another, thereby timings as 
within the normal playback are ignored!
