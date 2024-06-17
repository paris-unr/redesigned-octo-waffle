# misty_wrapper
### About

This is a ROS wrapper for the Misty II robot. It uses an expanded version of Misty Robotics's python wrapper to communicate with the Misty via local wireless. It is designed to allow multiple Mistys to be controlled via the same ROS server by specifying topics and nodes by id number (this requires launching misty.launch with appropriate arguments for each Misty).  

### Requirements
* ROS melodic
* python >3.8.0
* ffmpeg >4.0.0

### Usage
#### Recommended first-time setup:
```
cd [your_ROS_workspace]
git clone [this package] ./src/misty_wrapper
catkin_make
source devel/setup.bash
python3 -m venv isat_venv
source isat_venv/bin/activate
pip install -r [location_of_this_package]/requirements.txt
```
#### Launch
`roslaunch misty_wrapper misty.launch robot_ip:=None robot_id:=0 use_av_stream:=true`

### Nodes

* /misty_ROS_id_{#} : enables basic commands to be issued to Misty, currently including head (3 DOF) and arm (2 x 1 DOF) motor commands, the included Misty TTS, the image displayed on the face, and the LED color.
* /misty_AV_id_{#}  : includes video streaming and face recognition (only launched if arg use_av_stream = true)
* /mistyscan        : scans the local network for Mistys to connect to, or allows you to enter a known IP directly (only launched if arg robot_ip is not specified)

### Topics
_Subscribes to:_
| *Topic*               | *Msg type*        | *Description*  |
|---|---|---|
| /misty/id_{#}/speech  | std_msgs/String   | text-to-speech from SSML string  |
| /misty/id_{#}/arms    | msg/MoveArms      | rotation commands to arms  |
| /misty/id_{#}/head    | msg/MoveHead      | motor commands to head (RPY)  |
| /misty/id_{#}/face_img| std_msgs/String   | change Misty's face display to an image in memory by filename
| /misty/id_{#}/led     | std_msgs/Int8MultiArray | change Misty's LED color to \[R, G, B\] in range \[0, 256)

_Publishes to:_
| *Topic*               | *Msg type*        | *Description*  |
|---|---|---|
| /misty/id_{#}/camera  | sensor_msgs/Image | only if AV streaming enabled at launch  |
| /misty/id_{#}/face_recognition    | msg/DetectedFace   | face recognition info (if AV stream enabled)  |
