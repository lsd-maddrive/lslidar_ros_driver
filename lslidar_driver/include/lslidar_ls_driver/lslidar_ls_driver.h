/******************************************************************************
 * This file is part of lslidar_cx driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef LSLIDAR_Ch_DRIVER_H
#define LSLIDAR_Ch_DRIVER_H

#include <unistd.h>
#include <cstdio>
#include <netinet/in.h>
#include <string>
#include <memory>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/register_point_struct.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <chrono>
#include <tf/transform_listener.h>
#include <std_msgs/Int64.h>
#include <lslidar_msgs/LslidarChPacket.h>
#include <lslidar_msgs/LslidarChScanUnified.h>
#include <lslidar_msgs/LslidarChPacket.h>
#include <lslidar_msgs/LslidarChPoint.h>
#include <lslidar_msgs/LslidarChScan.h>
#include <lslidar_msgs/LslidarChSweep.h>
#include <lslidar_msgs/LslidarChLayer.h>
#include <deque>
#include <mutex>
#include "lslidar_ls_driver/ThreadPool.h"

#include "input.h"

namespace lslidar_ch_driver {
    /** Special Defines for LSCh support **/
    static const int POINTS_PER_PACKET_SINGLE_ECHO = 1192;       // modify
    static const int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;       // modify
    static float g_fDistanceAcc = 0.1 * 0.01f;
    static float m_offset = 6.37f;
    static double cos30 = cos(DEG2RAD(30));
    static double sin30 = sin(DEG2RAD(30));
    static double cos60 = cos(DEG2RAD(60));
    static double sin60 = sin(DEG2RAD(60));

    struct PointXYZIRT {
        PCL_ADD_POINT4D;
        PCL_ADD_INTENSITY;
        uint16_t ring;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

    struct Firing {
        double vertical_angle;
//        int vertical_line;
        double azimuth;
        double distance;
        float intensity;
        double time;
        int channel_number;
    };

    class LslidarChDriver {
    public:
        LslidarChDriver(ros::NodeHandle &n, ros::NodeHandle &pn);

        ~LslidarChDriver();

        bool initialize();

        bool polling();

        void difopPoll();

        void initTimeStamp();

        bool isPointInRange(const double &distance) const {
            return (distance >= min_range && distance <= max_range);
        }

        int convertCoordinate(const struct Firing &lidardata);

        // Publish data
        void publishPointCloudNew();

        void lslidarChPacketProcess(const lslidar_msgs::LslidarChPacketPtr &msg);

        typedef boost::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef boost::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

    public:
        bool loadParameters();

        bool createRosIO();

        //socket Parameters
        int msop_udp_port{};
        int difop_udp_port{};

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;

        // Converter convtor_
        std::shared_ptr<std::thread> difop_thread_;

        // Ethernet relate variables
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;

        in_addr lidar_ip{};
        int socket_id;
        bool add_multicast{};
        double prism_angle[4]{};

        // ROS related variables
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        bool use_time_service;
        uint64_t pointcloudTimeStamp{};
        unsigned char packetTimeStamp[10]{};
        ros::Time timeStamp;

        // Configuration parameters
        double min_range;
        double max_range;
        bool publish_point_cloud;
        ros::Time packet_timeStamp;
        double packet_end_time;
        double current_packet_time;
        double last_packet_time;

        std::string pointcloud_topic;
        ros::Publisher point_cloud_pub;
        int return_mode;
        int scan_start_angle{};
        int scan_end_angle{};
        double g_fAngleAcc_V;
        bool is_add_frame_;
        bool is_get_difop_;
        std::mutex pc_mutex_;
        ros::Publisher packet_loss_pub;
        int64_t last_packet_number_;
        int64_t current_packet_number_;
        int64_t tmp_packet_number_;
        int64_t total_packet_loss_;
        int frame_count;
        int m_horizontal_point = -1;
        bool get_ms06_param;
        //double m_offset;

        double cos_table[36000]{};
        double sin_table[36000]{};
        double cos_mirror_angle[4]{};
        double sin_mirror_angle[4]{};
        std::unique_ptr <ThreadPool> threadPool_;

        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_pub_;
    };
    typedef PointXYZIRT VPoint;
    typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
    typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;

//    typedef pcl::PointCloud<VPoint> VPointCloud;

} // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_ch_driver::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (double, timestamp, timestamp)
)

#endif // _LSLIDAR_Ch_DRIVER_H_
