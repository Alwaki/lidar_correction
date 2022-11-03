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


class LidarCorrectionNode
{
    public:
       LidarCorrectionNode();
       virtual ~LidarCorrectionNode();
       void node_thread();

    private:
        ros::NodeHandle                                 _nh;
        ros::Subscriber                                 _dji_imu_sub;
        ros::Subscriber                                 _dji_pos_sub;
        ros::Subscriber                                 _cloud_sub;
        ros::Publisher                                  _cloud_correct_pub;

        //tf2_ros::Buffer                                 _tf_buffer;
        //tf2_ros::TransformListener                      _tf_listener(tf2_ros::Buffer _tf_buffer);
        tf::TransformListener                           _tf_listener;
        tf2_ros::TransformBroadcaster                   _dynamic_broadcaster;
        tf2_ros::StaticTransformBroadcaster             _static_broadcaster;

        std::deque<std::pair<ros::Time, CloudXYZIT>>    _cloud_buf;
        std::mutex                                      _cloud_buf_mtx;
        Pos3D                                           _latest_position;
        std::mutex                                      _pos_mtx;
        uint32_t                                        _skip_counter;
        uint8_t                                         _cloud_counter;

        void _init_node();
        void _imu_callback(const sensor_msgs::Imu::ConstPtr &ImuMsg);
        void _pos_callback(const geometry_msgs::PointStamped::ConstPtr &PosMsg);
        void _cloud_callback(const livox_ros_driver::CustomMsg::ConstPtr &msgIn);
};


#endif