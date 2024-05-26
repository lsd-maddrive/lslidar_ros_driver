cmake_minimum_required(VERSION 2.8.3)
project(lslidar_ls180s2_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  LslidarChLayer.msg
  LslidarChPacket.msg
  LslidarChPoint.msg
  LslidarChScan.msg
  LslidarChSweep.msg
  LslidarChScanUnified.msg
)

add_service_files(
        FILES
        frame_rate.srv
        data_ip.srv
        destination_ip.srv
        data_port.srv
        dev_port.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
