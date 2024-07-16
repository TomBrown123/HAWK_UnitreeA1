<p align="center"><img src="docs/imgs/HAWK_I.png" width="25%" /></p>
<p align="center"> <font-size: 32px;"><strong>Unitree A1</strong></p>
  
#### User manual
- The official user manual for the Unitree A1 can be found [here](https://github.com/TomBrown123/HAWK_UnitreeA1/blob/main/docs/A1UserManual.pdf)

#### Accessing the A1 via LAN or the Wifi Hotspot

- The Nvidea NX IP address is `192.168.123.12`. This board handles SLAM, Image Transmission and Stereo Vision and is also where you will implement most of your code.
- The UP-CHT01 IP address is `192.168.123.161`. This board handles User Logic control, Sport mode controller and the unitree_sdk. 
- The default password for both is **123**
- The default password for the wifi hotspot is **00000000**
- If you are accessing from a remote PC then set your connection to the following
```bash
ip:      192.168.123.51
netmask: 24
gateway: 255.255.255.0
```

#### Connecting to the internet

- Connect the A1 via ethernet to a router
- Open a terminal on A1 and type `sudo gedit /etc/network/interfaces` and comment everything out.
- Save the file and in the terminal type `sudo systemctl restart NetworkManager.service`
- On the top right of the A1 desktop, change the network to **Wired connection**
- Now the internet should be working.
- In order to restore the connection between all the components again, undo the changes to the **interfaces** file and restart the network manager again.
- Sometimes a reboot is necessary.

#### Powering the A1

- I've made an **XT30** cable which can be used with a power supply to directly power the A1. Plug the cable into one of the XT30 connections on the right-hand side of the A1's back. The three XT30 pins on the left side are outputs for powering modules you want to connect to the A1.
- This is useful when implementing something on the A1 directly as the battery generally only lasts around 30 minutes.
- It should be noted that the A1 requires 24V with around 3A of current. At startup it may take up to 20A of current. If the power supply you are using is underpowered and you try to make it walk or do any other movement based action, the A1 will go into safe mode and crash to the ground .

####  Unitree_legged_sdk

- The Unitree_legged_sdk contains some examples for High and Low level control.
- For High level control, make sure the robot is standing and for Low level control the robot must be hung from a strap.
- These examples communicate with the A1 over UDP and can be run with the relevant command, such as `sudo ./example_walk`
- However, when I ran these examples, the A1 went through the movements briefly and then fell to the ground. 

#### RobotSLAMSystem

- The `RobotSLAMSystem` will be begin on startup if the lidar is connected to the Nvidea NX, otherwise the RobotVisionSystem will start.
- Both the RobotSLAMSystem and the RobotVisionSystem cannot be run at the same time. Attempting this will result in a `bind client ip&port failed` error for whichever process starts second. Thus, you must first `kill PID # replace PID with RobotVisionSyst PID` if RobotVisionSystem is running.
- To start the RobotSLAMSystem, navigate to the **RobotSLAMSystem/slamplanner** folder in the terminal and type `./start.sh`. I have modified this file to also launch the **qre_a1/a1_hardware_driver/high_level_mode.launch** file, which subscribes to cmd_vel and publishes to an LCM server on port 8080. The original `start.sh` is saved as `start_original.sh`. I was never able to get the A1 to move using the original start.sh file, although the `cmd_vel` topic was publishing to the `/base_controller_node`
- Rviz will also launch automatically with the relevant config. This can be disabled by commenting out the Rviz nodelet in **~/catkin_ws/src/slamrplidar/slamplanner/launch/slam_planner_online.launch**
- You can set a goal using the 2d Nav Goal button in Rviz or by directly sending a command to **/move_base_simple/goal**
- However, this is still not working perfectly. The connection to the lidar sensor sometimes fails and a restart is necessary. Another problem I encounted was that sometimes the LaserScan and local costmap weren't receiving any data, which can be checked using `rostopic echo /scan` and `rostopic echo /slam_planner_node/local_costmap/costmap`. If Rviz is running you will also see that there is just a white square where the local costmap should be. 

#### RobotVisionSystem

- This uses the MJPG streamer to stream video through http to the unitree mobile app. You can access this stream through the web browser by typing `192.168.123.12:8080/`
- If you want to use this stream with opencv or something similar, then use `http://192.168.123.12:8080/?action=stream`.
- With opencv for example, this would be: `cap = cv.VideoCapture('http://192.168.123.12:8080/?action=stream')`

#### qre_a1 package from MYROBOTSHOP

- This is a private repository from MYBOTSHOP which Niklas has access to.
- Here is a [link](https://www.docs.quadruped.de/projects/a1/html/quick_start.html#robot-setup) to the MYBOTSHOP guide on using the A1 and their qre_a1 package
- The **unitree_legged_sdk** (/utils) and the **ros_to_real** (/third_party) packages are already included in the qre repository. Please refer to the readme for installing.
- This package requires an Ouster and ZED2 camera for full functionality, such as SLAM and autonomous navigation. However, some of the launch files can still be used with the setup on our A1 and it is possible to get the odom data required for the package from another camera using something like [VINS_Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion?tab=readme-ov-file) or the [realsense ROS-wrapper](https://github.com/TomBrown123/realsense-ros/tree/ros1-legacy).

The following will allow you to control the A1's movement with a keyboard.

```bash
sudo su
source catkin_ws/devel/setup.bash
roslaunch a1_hardware_driver high_level_mode.launch # This launches an LCM server that communicates with the UP-CHT01
rosrun teleop_twist_keyboard teleop_twist_keyboard.py # Keyboard control
```

#### Gazeebo simulation

- Usage of the qre_a1 and Unitree Gazeebo simulation is detailed [here](https://www.docs.quadruped.de/projects/a1/html/simulation.html). However, these are incapable of High Level control, such as walking.
- For simulation of the A1 I would recommend using [CHAMP](https://github.com/chvmp/champ). You can find the config files for the A1 [here](https://github.com/chvmp/robots/tree/master/configs/a1_config)

#### Gait Planner

- It is also possible to implement new gaits for the A1 using CHAMP, TOWR or Free Gait. MYBOTSHOP has published a short guide on this [here](https://www.mybotshop.de/QUADRUPED-Gait-Planning)
##### Here are the links to the different packages for gait planning:
- [CHAMP Setup Assistant](https://github.com/chvmp/champ_setup_assistant)
- [TOWR](https://wiki.ros.org/towr)
- [Free Gait](https://github.com/leggedrobotics/free_gait)

#### librealsense SDK:

- This is the SDK for the D435i Camera.
- If you want to use the SDK you must first kill the `RobotVisionSyst` process.
- Type `realsense-viewer` into the terminal to access the realsense viewer program. Here you can view all the different possible camera streams from the d435i as well as a 3d pointcloud stream.
- You can also upgrade the camera firmware through this software, but it is currently set to the correct firmware for the **librealsense** SDK and **ROS-Wrapper** version installed on the A1.
- The **Realsense SDK** also includes some examples which are already compiled in the **/home/unitree/librealsense/build/examples** folder
- To run them, just type the name of the file in the terminal, for example `rs-capture` which launches a window with a colour, depth, gyro and accelerometer stream
- Each example also has a [readme](https://github.com/TomBrown123/librealsense/tree/master/examples) file containing a code break down.
-  librealsense also has a number of tools, such as `rs-enumerate-devices`, which prints a list of information about the camera to the terminal, such as the firmware version and what streaming profiles are available.
- A full list of the tools can be found [here](https://github.com/TomBrown123/librealsense/tree/master/tools)
#### Realsense ROS-wrapper

- The ros-wrapper is located in **/home/unitree/catkin_ws/src/realsense/realsense2_camera** and also contains many example roslaunch files.
- Type `roslaunch realsense2_camera rs_capture` into the terminal to stream all camera sensors and publish them on the appropriate ROS topics.
- These streams can then be visualised through Rviz.
- Refer to the **realsense2_camera** [readme](https://github.com/TomBrown123/realsense-ros/tree/ros1-legacy) for a detailed guide on its usage.

#### The Gesture Recognition demo

- First, open the readme file and install all the required dependencies. The [readme](https://github.com/TomBrown123/hand-gesture-recognition-mediapipe) also contains a detailed explaination of how the program functions and how you can train the model to recognize new hand gestures
- Next, connect to the A1 with your remote pc via ethernet or the A1 wifi hotspot.
- Then open a terminal, navigate to the folder containing the app.py file and type `python3 app.py`
- A window should now pop up with the video stream which recognizes some hand gestures.

#### rbd_packages

- This package was also built using an Ouster and ZED2, and as such needs to first be modified in order to work. Unfortunately there is a header file that they placed in their **qre_a1** package which is not included on their github. The **rbd_ros** package, which is supposed to used on the robot, cannot be built with catkin_make as long as it is missing.
- However, along with the [Bachelor thesis](https://www.zhaw.ch/storage/engineering/institute-zentren/cai/studentische_arbeiten/Spring_2023/Spring23_BA_PfammatterSchweizer.pdf) describing how it was designed, it provides a good start for implementing gesture recognition based movements on the A1 if one decided to build it from scratch.
- There are some unitree SDK files which they created located [here](https://github.com/TomBrown123/rbd_packages/tree/main/rbd_local/zhaw_ba_robodog/example_files), which can be placed in the **unitree_legged_SDK** examples folder. They will need to be compiled first with cmake.
