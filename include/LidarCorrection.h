#ifndef LIDARCORRECTION_H
#define LIDARCORRECTION_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "livox_ros_driver/CustomMsg.h"

#include <deque>
#include <thread>
#include <string>
#include <stdio.h>

struct PointXYZIT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint32_t t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity) (uint32_t, t, t))

typedef geometry_msgs::PointStamped         Pos3D;
typedef sensor_msgs::Imu                    Rot3D;
typedef pcl::PointCloud<PointXYZIT>         CloudXYZIT;
typedef pcl::PointCloud<PointXYZIT>::Ptr    CloudXYZITPtr;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::Imu> MySyncPolicy;


class LidarCorrectionNode
{
    public:
       LidarCorrectionNode();
       virtual ~LidarCorrectionNode();
       void node_thread();

    private:
        ros::NodeHandle                                             _nh;
        ros::Subscriber                                             _cloud_sub;
        ros::Publisher                                              _cloud_correct_pub;

        message_filters::Subscriber<geometry_msgs::PointStamped>    _dji_pos_sub;
        message_filters::Subscriber<sensor_msgs::Imu>               _dji_imu_sub;
        message_filters::Synchronizer<MySyncPolicy>                 _dji_sync;

        tf::TransformListener                                       _tf_listener;
        tf2_ros::TransformBroadcaster                               _dynamic_broadcaster;

        std::deque<std::pair<ros::Time, CloudXYZIT>>                _cloud_buf;
        std::mutex                                                  _cloud_buf_mtx;

        void _init_node();
        void _dji_callback(const geometry_msgs::PointStamped::ConstPtr &PosMsg, const sensor_msgs::Imu::ConstPtr &ImuMsg);
        void _cloud_callback(const livox_ros_driver::CustomMsg::ConstPtr &msgIn);
};


#endif