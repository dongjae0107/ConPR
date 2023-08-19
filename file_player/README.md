# File Player for ConPR Dataset

Maintainer: Dongjae LEE (dongjae0107@gamil.com)

## 1. Prerequisites 
### 1.1 Ubuntu and ROS
For the "Desktop-Full version" ROS, the default ROS packages are enough for File_Player. Only one additional dependent package(follow 1.2 livox_ros_driver) is needed. 
### 1.2 livox_ros_driver
Follow <a href="https://github.com/Livox-SDK/livox_ros_driver"><strong>livox_ros_driver_installation</strong></a>.

**livox_ros_driver** must be installed and sourced before building and running File_Player.

## 2. Build

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/dongjae0107/ConPR.git
cd ..
catkin_make
source devel/setup.bash
```

## 3. Run File Player

```
roslaunch file_player file_player.launch
```

## 4. Load Data and Play

1. Click 'Load' button.
2. Choose data folder including 'data_stamp.csv' file, 'LiDAR' folder, 'Camera' folder and 'Calibration' folder.
3. Click 'Play' button to start publishing the data in ROS message.

## 5. Acknowledgement
Original author: Jinyong Jeong (jjy0923@kaist.ac.kr)

Original repository: <a href="https://github.com/irapkaist/file_player">file_player</a>