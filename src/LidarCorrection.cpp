#include "LidarCorrection.h"

LidarCorrectionNode::LidarCorrectionNode():
    _nh("~")
    {_init_node();}

LidarCorrectionNode::~LidarCorrectionNode() = default;

void LidarCorrectionNode::_init_node()
{
    // Skip counter is used to delay start of lidar. Cloud counter is for aggregation of clouds.
    _skip_counter = 0;
    _cloud_counter = 0;

    // Create static transform between drone and sensor
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "drone";
    static_transformStamped.child_frame_id = "livox_frame";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.w = 0.7071;
    static_transformStamped.transform.rotation.x = 0;
    static_transformStamped.transform.rotation.y = 0.7071;
    static_transformStamped.transform.rotation.z = 0;
    _static_broadcaster.sendTransform(static_transformStamped);

    // Setup subscribers and publishers
    _dji_imu_sub       = _nh.subscribe<sensor_msgs::Imu>("/dji_osdk_ros/imu", 400, 
                                                         &LidarCorrectionNode::_imu_callback, this);
    _dji_pos_sub       = _nh.subscribe<geometry_msgs::PointStamped>("/dji_osdk_ros/local_position",
                                                                    50, &LidarCorrectionNode::_pos_callback,
                                                                    this);
    _cloud_sub         = _nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, 
                                                          &LidarCorrectionNode::_cloud_callback, 
                                                          this, ros::TransportHints().tcpNoDelay(true));
    _cloud_correct_pub = _nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_corrected", 10);
}

void LidarCorrectionNode::node_thread()
{
    CloudXYZIT corrected_aggregated_cloud;
    while(true)
    {
        if(!_cloud_buf.empty())
        {
            // Get cloud from buffer and remove it
            _cloud_buf_mtx.lock();
            auto base_time = _cloud_buf.front().first;
            auto cloud = _cloud_buf.front().second;
            _cloud_buf.pop_front();
            _cloud_buf_mtx.unlock();

            // Create new cloud
            size_t oldsize = corrected_aggregated_cloud.points.size();
            size_t cloudsize = cloud.points.size();
            corrected_aggregated_cloud.points.resize(oldsize + cloudsize);

            // Fill new cloud with corrected points
            for (size_t i = 0; i < cloudsize; i++)
            {
                ros::Duration time_offset(cloud.points[i].t*1.0e-9);
                auto time = base_time + time_offset;
                geometry_msgs::PointStamped p_in, p_out;
                p_in.point.x = cloud.points[i].x;
                p_in.point.y = cloud.points[i].y;
                p_in.point.z = cloud.points[i].z;
                p_in.header.stamp = time;
                p_in.header.frame_id = "livox_frame";
                _tf_listener.waitForTransform("livox_frame", "world", time, ros::Duration(0.02));
                _tf_listener.transformPoint("world", p_in, p_out);
                corrected_aggregated_cloud.points[oldsize + i].x = p_out.point.x;
                corrected_aggregated_cloud.points[oldsize + i].y = p_out.point.y;
                corrected_aggregated_cloud.points[oldsize + i].z = p_out.point.z;
                corrected_aggregated_cloud.points[oldsize + i].t = cloud.points[i].t;
                corrected_aggregated_cloud.points[oldsize + i].intensity = cloud.points[i].intensity;
            }

            // If enough clouds aggregated, publish corrected cloud
            _cloud_counter++;
            if(_cloud_counter > 5)
            {
                _cloud_counter = 0;
                sensor_msgs::PointCloud2 tempCloud;
                pcl::toROSMsg(corrected_aggregated_cloud, tempCloud);
                tempCloud.header.stamp = ros::Time(base_time);
                tempCloud.header.frame_id = "world";
                _cloud_correct_pub.publish(tempCloud);
                corrected_aggregated_cloud = {};
            }
            
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}

void LidarCorrectionNode::_imu_callback(const sensor_msgs::Imu::ConstPtr &ImuMsg)
{
    // Set rotation transform of drone from IMU message
    geometry_msgs::TransformStamped transform;
    transform.transform.rotation.w = ImuMsg->orientation.w;
    transform.transform.rotation.x = ImuMsg->orientation.x;
    transform.transform.rotation.y = ImuMsg->orientation.y;
    transform.transform.rotation.z = ImuMsg->orientation.z;

    // Set translation transform of drone from latest known position
    _pos_mtx.lock();
    transform.transform.translation.x = _latest_position.point.x;
    transform.transform.translation.y = _latest_position.point.y;
    transform.transform.translation.z = _latest_position.point.z;
    _pos_mtx.unlock();

    // Publish pose transform of drone
    transform.header.stamp = ImuMsg->header.stamp;
    transform.child_frame_id = "drone";
    transform.header.frame_id = "world";
    _dynamic_broadcaster.sendTransform(transform);
}

void LidarCorrectionNode::_pos_callback(const geometry_msgs::PointStamped::ConstPtr &PosMsg)
{
    // Zero position of drone by removing initial position
    static geometry_msgs::PointStamped first_point = *PosMsg;

    // Make latest translational position of drone available
    geometry_msgs::PointStamped point;
    point.header = PosMsg->header;
    point.header.frame_id = "local";
    point.point = PosMsg->point;

    point.point.x -= first_point.point.x;
    point.point.y -= first_point.point.y;
    point.point.z -= first_point.point.z;
    _pos_mtx.lock();
    _latest_position = point;
    _pos_mtx.unlock();
}

void LidarCorrectionNode::_cloud_callback(const livox_ros_driver::CustomMsg::ConstPtr &msgIn)
{
    // Skip first few lidar clouds (to avoid problem when frames do not exist yet)
    _skip_counter++;
    if(_skip_counter>10)
    {
        // Change cloud to PCL structure and push to queue
        size_t cloudsize = msgIn->points.size();
        CloudXYZIT cloud;
        cloud.points.resize(cloudsize);

        for (size_t i = 0; i < cloudsize; i++)
            {
                auto &src = msgIn->points[i];
                auto &dst = cloud.points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.reflectivity;
                dst.t = src.offset_time;
            }
        _cloud_buf_mtx.lock();
        _cloud_buf.push_back(std::make_pair(msgIn->header.stamp, cloud));
        _cloud_buf_mtx.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh("~");

    ROS_INFO("----> lidar node started");

    LidarCorrectionNode node;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    node.node_thread();
    ros::waitForShutdown();
}