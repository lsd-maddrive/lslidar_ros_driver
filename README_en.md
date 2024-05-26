# LSLIDAR_LS1500_ROS

## 1.Introduction
​		LSLIDAR_LS1550_ROS  is the lidar ros driver in linux environment, which is suitable for   LS128S1、LS128S2、LS25D、LS180S1、LS180S2 and LS400 、MS06   lidar. The program has  tested under ubuntu 20.04 ros noetic and ubuntu18.04 ros melodic.

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 16.04**: ros-kinetic-desktop-full

**Ubuntu 18.04**: ros-melodic-desktop-full

**Ubuntu 20.04**: ros-noetic-desktop-full

**Installation**: please refer to [http://wiki.ros.org](http://wiki.ros.org/)

2.ros dependencies

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.other dependencies

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #Select the appropriate version
~~~

## 3.Compile & Run

### 3.1 Compile

~~~bash
mkdir -p ~/lidar_ws/src
~~~

Copy the whole lidar ROS driver directory into ROS workspace, i.e "~/lidar_ws/src".

~~~bash
cd ~/lidar_ws
catkin_make
source devel/setup.bash
~~~

3.2 Run

run with single lidar:

~~~bash
roslaunch lslidar_ls180s2_driver lslidar_ls1550.launch
~~~

run with double lidar:

~~~bash
roslaunch lslidar_ls180s2_driver lslidar_ls1550_double.launch
~~~

## 4. Introduction to parameters

The content of the lslidar_ls128.launch file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
<launch>
  <arg name="device_ip" default="192.168.1.200" />     #lidar ip
  <arg name="msop_port" default="2368" />              # Main data Stream Output Protocol packet port
  <arg name="difop_port" default="2369" />             # Device Information Output Protocol packet port
  <arg name="use_time_service" default="false" />  # Whether use time service
  <arg name="packet_rate" default="15000.0"/>     #PCAP packets per second, used for offline parsing of PCAP packets

    <node pkg="lslidar_ls180s2_driver" type="lslidar_ls180s2_driver_node" name="lslidar_ls180s2_driver_node" output="screen">
<!--param name="pcap" value="$(find lslidar_ls128_driver)/pcap/xxx.pcap"/-->
#Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
    <param name="lidar_ip" value="$(arg device_ip)"/>
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="add_multicast" value="false"/>              # Whether to add multicast
    <param name="group_ip" value="224.1.1.2"/>               #multicast ip
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="frame_id" value="laser_link"/>              # lidar point cloud coordinate system name
    <param name="min_range" value="0.15"/>                   # Minimum measuring distance of lidar
    <param name="max_range" value="500.0"/>                  # Maximum measuring distance of lidar
    <param name="scan_start_angle" value="-60"/>             #Scan start angle, range - 60 ° to 60 °
    <param name="scan_end_angle" value="60"/>                #Scan end angle,   range - 60 ° to 60 °
    <param name="pointcloud_topic" value="lslidar_point_cloud"/>  # name of point cloud topic
    <param name="packet_loss" value="false"/>   #Is packet loss detection enabled
    <param name="time_synchronization" value="$(arg time_synchronization)"/>  # Whether gps time synchronization
    
  </node>

  <!--node name="rviz" pkg="rviz" type="rviz" args="-d $(find lslidar_ls180s2_driver)/launch/lslidar_ls.rviz" output="screen"/-->

</launch>
~~~

## 5. Packet loss detection

Drive to publish the total number of lidar packet loss in the form of topic with the name of "packet_loss" and the message type of std_ msgs::msg::Int64。



# 6.ROS服务

### Set lidar frame rate:

Open a new terminal  

source devel/setup.bash

Optional frame rate Normal frame rate/50 percent frame rate/25 percent frame rate

~~~bash
rosservice call /set_frame_rate "frame_rate: 0" 		#Normal frame rate
rosservice call /set_frame_rate "frame_rate: 1"			#50 percent frame rate
rosservice call /set_frame_rate "frame_rate: 2"			#25 percent frame rate
~~~



### Set lidar IP:

Open a new terminal  

~~~bash
source devel/setup.bash
rosservice call /set_data_ip "data_ip: ''"		
#Fill in the IP address in '', for example: 192.168.1.201
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar destination IP:

Open a new terminal  

~~~bash
source devel/setup.bash
rosservice call /set_destination_ip "destination_ip: ''"
#Fill in the lidar destination IP in '', for example: 192.168.1.100
~~~

**Note: After setting, you need to modify the Modify local IP address and restart the driver.**



### Set lidar destination data port:  

Open a new terminal  

~~~bash
source devel/setup.bash
rosservice call /set_data_port "data_port: " 
#Fill in the lidar destination device port in the field, for example: 2370
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar destination device port :

Open a new terminal  

~~~bash
source devel/setup.bash
rosservice call /set_dev_port "dev_port: "
#Fill in the lidar destination device port in the field, for example: 2371
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



## FAQ

Bug Report

Original version : LSLIDAR_LS128_ROS_V1.0.0_220622_ROS

Modify:  original version

Date    : 2022-06-24

-----------------------------------------------------------------------------------------
Original version : LSLIDAR_LS128_ROS_V1.0.1_220805_ROS

Modify:  1. Fix the problem of missing horizontal angle

Date    : 2022-08-05

-----------------------------------------------------------------------------------------

Original version : LSLIDAR_LS128_ROS_V1.0.2_220901_ROS

Modify:  1. Add GPS time service and PTP time service timestamp analysis

Date    : 2022-09-01

----------------------

Original version : LSLIDAR_LS128_ROS_V1.0.3_220929_ROS

Modify:  1. Upgrade the fpga version, and change the judgment condition of the device package header

2. Modify the timestamp calculation method of the point
2. Modify the calculation formula of the galvanometer offset angle fGalvanometrtAngle

Date    : 2022-09-29

-----------------------

Original version : LSLIDAR_LS128_ROS_V1.0.4_221128_ROS

Modify:  1. Upgrade the fpga version, and modify pointcloud calculation formula

Date    : 2022-11-28

------------------

Original version : LSLIDAR_LS128_ROS_V1.0.5_230227_ROS

Modify:  1. Added support for LS128S2 lidar

Date    : 2023-02-27

---------------------

Original version : LSLIDAR_LS128_ROS_V1.0.6_230301_ROS

Modify:  1. Change the fpga protocol and modify the overlapped frame judgment condition

Date    : 2023-03-01

-----------------

Original version : LSLIDAR_LS128_ROS_V1.0.7_230313_ROS

Modify:  1. Calculate the sine and cosine values of horizontal and vertical angles in advance to reduce CPU consumption

​                2. Add packet loss detection function

Date    : 2023-03-13

---

Original version : LSLIDAR_LS1550_ROS_V1.0.8_230907_ROS

Modify:  

1. Add compatible MS06 radar

2. Fix packet loss detection topic data anomalies

3. Modified the angle and offset values of the radar pendulum mirror

Date    : 2023-09-07

----

Original version : LSLIDAR_LS1550_ROS_V1.0.9_240301_ROS

Modify:  

1. optimized code 

​	2.Add and modify radar frame rate, IP, and port services

Date    : 2024-03-01

----

