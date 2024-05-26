# LSLIDAR_CX_V4.3.1_240409_ROS illustrate

## 1.Introduction

​		LSLIDAR_CX_V4.3.1_240409_ROS  is a lidar ROS driver in a Linux environment. The program has been tested successfully under ubuntu 20.04 ROS nonetic, ubuntu18.04 ROS media, and ubuntu16.04 ROS kinetic.

Applicable to C16,C32 3.0 version and C1, C1Plus, C4, C8, C8f, CKM8, C16, MSC16, C16 domestic, C32, C32W, C32WN,C32Wb,C32WP, CH32R 4.0 version and n301 5.5 version lidars.

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 16.04**: ros-kinetic-desktop-full

**Ubuntu 18.04**: ros-melodic-desktop-full

**Ubuntu 20.04**: ros-noetic-desktop-full

**Installation**: please refer to [http://wiki.ros.org]

2.ros dependencies

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions  ros-$ROS_DISTRO-diagnostic-updater
```

3.other dependencies

~~~bash
sudo apt-get install libpcap-dev 
~~~

## 3.Compile && Run

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

roslaunch lslidar_c16_driver lslidar_cx.launch

~~~

run with double lidar:

~~~bash
roslaunch lslidar_c16_driver lslidar_double.launch
~~~



## 4. Introduction to parameters

The content of the lslidar_cx.launch file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
<launch>
   <arg name="device_ip" default="192.168.1.200" />    # lidar ip address
  <arg name="msop_port" default="2368"/>   # Main data Stream Output Protocol packet port
  <arg name="difop_port" default="2369"/>  # Device Information Output Protocol packet port
  <arg name="use_time_service" default="false" />      # Whether time synchronization
  <arg name="pcl_type" default="false" />  # pointcloud type，false: xyzirt,true:xyzi

  <arg name="packet_rate" default="1695.0"/>           #The number of data packets sent by the lidar per second. This parameter is useful when reading pcap packets

  <node pkg="lslidar_c16_driver" type="lslidar_c16_driver_node" name="lslidar_driver_node" output="screen">
    <!--param name="pcap" value="$(find lslidar_driver)/pcap/123.pcap" /-->   #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="packet_rate" value="$(arg packet_rate)"/>
    <param name="device_ip" value="$(arg device_ip)" />
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="pcl_type" value="$(arg pcl_type)"/>
    <param name="add_multicast" value="false"/>     # Is multicast enabled	ture: enable
    <param name="group_ip" value="224.1.1.2"/>      # Multicast ip address
    <param name="frame_id" value="laser_link"/>     # lidar point cloud coordinate system name
    <param name="pointcloud_topic" value="lslidar_point_cloud"/> #point cloud topic name, can be modified
    <param name="distance_min" value="0.15"/>       # Unit: m. The minimum value of the lidar blind area, points smaller than this value are filtered
    <param name="distance_max" value="200.0"/>      # Unit: m. The maximum value of the lidar blind area, points smaller than this value are filtered
    <param name="angle_disable_min" value="0"/>     # lidar clipping angle start value ，unit:0.01°
    <param name="angle_disable_max" value="0"/>     # lidar clipping angle end value ，unit:0.01°
    <param name="distance_unit" value="0.40"/>
    <param name="horizontal_angle_resolution" value="0.18"/>  # Horizontal angle resolution 10Hz:0.18  20Hz:0.36 5Hz: 0.09 5Hz:0.09-->
    <param name="publish_scan" value="false"/>      # Whether to publish the scan
    <param name="scan_num" value="15"/>             # laserscan line number
    <param name="coordinate_opt" value="false"/>    # Default false. The zero degree angle of the lidar corresponds to the direction of the point cloud
  </node>

  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find lslidar_c16_driver)/rviz/lslidar.rviz"/>			   # If it is not necessary to start rivz,please delete this line
 
 <!--node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 0 0 0 0 world laser_link 100" /-->					# tf coordinate conversion
  
</launch>
~~~

- ### Multicast mode:

  - The host computer sets the lidar to enable multicast mode

  - Modify the following parameters of the launch file

  ~~~shell
  <param name="add_multicast" value="true"/>                 # add multicast
  <param name="group_ip" value="224.1.1.2"/>                 # The multicast ip address set by the host computer
  ~~~

- Run the following command to add the computer to the group (replace enp2s0 in the command with the network card name of the user's computer, you can use ifconfig to view the network card name)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



- ### Offline pcap mode:

  - Copy the recorded pcap file to the lslidar_c16_driver/pcap folder
  
  - Modify the following parameters of the launch file
  
  ~~~shell
  // uncomment
      <param name="pcap" value="$(find lslidar_c16_driver)/pcap/xxx.pcap" /> 
  ~~~

- ### pcl point cloud type:

  - Modify the following parameters of the launch file

  ~~~shell
  <param name="pcl_type" value="$(arg pcl_type)"/>          # pointcloud type，false: xyzirt,true:xyzi
  ~~~

- The default false is the custom point cloud type, which references the definition in the file of

  lslidar_driver/include/lslidar_driver.h

  Change it to true, which is the own type of pcl:

  ~~~c++
  pcl::PointCloud<pcl::PointXYZI>
  ~~~

### Modify lidar time service mode:

source devel/setup.bash

GPS：

~~~bash
rosservice call /time_service "time_service_mode: 'gps'
ntp_ip: ''" 
~~~

PTP：

Only supports 4.0 lidar

~~~bash
rosservice call /time_service "time_service_mode: 'ptp'
ntp_ip: ''" 
~~~

NTP：

Only supports 4.0 lidar

~~~bash
rosservice call /time_service "time_service_mode: 'ntp'
ntp_ip: '192.168.1.102'" 
~~~



### lidar power on/off(lidar still rotates,  only send equipment packets):

source devel/setup.bash

power on：

~~~bash
rosservice call /lslidar_control "laser_control: 1"
~~~

power off

~~~bash
rosservice call /lslidar_control "laser_control: 0"
~~~



### lidar rotates/stops rotating (motor stops rotating)：

source devel/setup.bash

rotate：

~~~bash
rosservice call /motor_control "motor_control: 1"
~~~

stop rotating:

~~~bash
rosservice call /motor_control "motor_control: 0"
~~~



### Set lidar speed:

source devel/setup.bash

Optional frequency  5Hz/10Hz/20Hz

~~~bash
rosservice call /set_motor_speed "motor_speed: 20"
~~~



### Set lidar  to remove rain, fog, and dust levels:

Only supports 4.0 lidar

source devel/setup.bash

Optional level 0/1/2/3, the larger the number from 0 to 3, the stronger the removal

~~~bash
rosservice call /remove_control "remove_control: 0" 
~~~



### Set lidar data packet port

source devel/setup.bash

~~~bash
rosservice call /set_data_port "data_port: 2368"  #range [1025,65535]
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar equipment packet port

source devel/setup.bash

~~~bash
rosservice call /set_dev_port "dev_port: 2369"  #range[1025,65535]
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar ip

source devel/setup.bash

~~~bash
rosservice call /set_data_ip "data_ip: '192.168.1.200'"
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar destination ip

source devel/setup.bash

~~~bash
rosservice call /set_destination_ip "destination_ip: '192.168.1.102'"
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



## FAQ

Bug Report

Original version : LSLIDAR_CX_V4.1.4_220425_ROS

Modify:  original version

Date    : 2022-04-25

--------------------------------------------------------------------



Update version : LSLIDAR_CX_V4.1.5_220620_ROS

Modify: 1.Modify the optical vertical angle.

Date    : 2022-06-20

------------------------------------------------------------



Update version : LSLIDAR_CX_V4.2.0_221028_ROS

Modify:  1. Added support for version 4.0 single line lidar and C32 70 degree and 90 degree lidars.

​				2.Unify laserscan and pointcloud2 coordinate systems.

Date    : 2022-10-28

-------------------



Update version : LSLIDAR_CX_V4.2.1_221227_ROS

Modify: 1. Scan topic adds strength information.

2. fpga upgrade, C32 90 degree modification of calculation formula.
3. ROS driver adds the function of modifying time service mode.
3. New function to modify lidar configuration.
3. Fixed the ntp timing resolution problem.

Date    : 2022-12-27

----------------------



Update version : LSLIDAR_CX_V4.2.2_230322_ROS

Modify:1.Prompt for usage duration.

​			 2.Driver version prompt.

Date    : 2023-03-22

--------------



Update version : LSLIDAR_CX_V4.2.3_230403_ROS

Modify:  1. Change the fpga protocol and modify the calculation formula of C32W.

Date    : 2023-04-03

------



Update version : LSLIDAR_CX_V4.2.4_230705_ROS

Modify:  1.Fix the issue of lidar switching to low power mode and not being able to switch to normal mode.

​               2. Compatible with C1Plus lidar models.

Date    : 2023-07-05

---



Update version : LSLIDAR_CX_V4.2.5_230913_ROS

Modify: 

1. Optimize the code to reduce CPU usage.
2. Add nodelet function.
2.  Delete the radar model parameters and rewrite the automatic recognition radar model.

Date    : 2023-09-13

---



Update version : LSLIDAR_CX_V4.2.6_231012_ROS

Modify: 

1. Compatible with n301 lidar models.

Date    : 2023-10-12

---



Update version : LSLIDAR_CX_V4.2.7_231020_ROS

Modify: 

1.Add the function of setting radar to remove rain, fog, and dust levels.

Date    : 2023-10-20

---



Update version : LSLIDAR_CX_V4.2.8_240321_ROS

Modify:  1.Compatible with C4 lidar.

​			  	2.Compatible with CH32R v4.8 lidar.

​		  		3.Change point cloud angle cropping to point cloud angle preservation.

​		  		4.Optimize the code to reduce CPU usage.

Date    : 2024-03-21

-----



Update version : LSLIDAR_CX_V4.2.9_240325_ROS

Modify:  1.Compatible with C16 C32 v3.0 version lidar.

​			   2.Compatible with C32WN and C32WB lidar.

​		  	 3.Change point cloud angle preservation to point cloud angle cropping (supports negative angle cropping).

Date    : 2024-03-25

-----



Update version : LSLIDAR_CX_V4.3.0_240401_ROS

Modify:  1.Fix 3.0 lidar power on/off failure issue.

​			   2.Restrict the range of lidar IP settings and prohibit setting 224 network segments as lidar IP.

Date    : 2024-04-01

-----



Update version : LSLIDAR_CX_V4.3.1_240409_ROS

Modify:  1.After data interruption and reacquiring data packets,re evaluating lidar model

​			   2.Fix the issue of continuous zero crossing angle between two adjacent data packet points in 3.0 lidar.

Date    : 2024-04-09

-----



