# HAWK Unitree A1

#### Connecting to the internet

- Connect the A1 via ethernet to the internet
- Open a terminal on A1 and type **sudo gedit /etc/network/interfaces** and comment everything out.
- Save the file and in the terminal type **sudo systemctl restart NetworkManager.service**
- On the top right of the desktop change the network to **Wired connection**
- Now the internet should be working.
- In order to restore the connection between all the components again, undo the changes to the **interfaces** file and restart the network manager again.
- Sometimes a reboot is necessary.

#### Using the librealsense sdk:

- Type **realsense-viewer** into the terminal to access the realsense viewer program. Here you can view all the different possible camera streams from the d435i as well as 3d pointcloud stream.
- You can also upgrade the camera firmware through this software, but it is currently set to the correct firmware for the librealsense SDK and ROS-Wrapper version installed on the A1.
- The **Realsense SDK** also includes some examples which are already compiled in the **/home/unitree/librealsense/build/examples** folder
- To run them, just type the name of the file in the terminal, for example **rs-capture** which launches a window with a colour, depth, gyro and accelerometer stream
- Each example also has a readme file containing a code break down found in **/home/unitree/librealsense/examples**
  
#### Using the realsense ros-wrapper

- The ros-wrapper is located in **/home/unitree/catkin_ws/src/realsense/realsense2_camera** and also contains many example roslaunch files.
- Type **roslaunch realsense2_camera rs_capture** into the terminal to stream all camera sensors and publish on the appropriate ROS topics.
- These streams can then be accessed and visualised through Rviz.
- Refer to the realsense2_camera readme for a detailed guide on the usage of the ROS Wrapper.
