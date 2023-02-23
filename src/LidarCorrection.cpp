#include "LidarCorrection.h"
#include "GaussConformalProjection.h"

LidarCorrectionNode::LidarCorrectionNode():
    _nh("~"),
    _dji_sync(MySyncPolicy(10), _dji_pos_sub, _dji_imu_sub)
    {_init_node();}

LidarCorrectionNode::~LidarCorrectionNode() = default;

void LidarCorrectionNode::_init_node()
{

    // Setup subscribers and publishers
    _cloud_sub         = _nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, 
                        &LidarCorrectionNode::_cloud_callback, this, ros::TransportHints().tcpNoDelay(true));
    _rtk_sub           = _nh.subscribe<sensor_msgs::NavSatFix>("/dji_osdk_ros/rtk_position", 10, 
                        &LidarCorrectionNode::_rtk_callback, this, ros::TransportHints().tcpNoDelay(true));
    _cloud_correct_pub = _nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_corrected", 10);

    _dji_pos_sub.subscribe(_nh, "/dji_osdk_ros/local_position", 1);
    _dji_imu_sub.subscribe(_nh, "/dji_osdk_ros/imu", 1);
    _dji_sync.registerCallback(boost::bind(&LidarCorrectionNode::_dji_callback, this, _1, _2));
}

void LidarCorrectionNode::node_thread()
{
    // Keep thread running
    while(true)
    {
        // Check cloud buffer
        if(!_cloud_buf.empty())
        {
            // Get cloud from buffer and remove it
            _cloud_buf_mtx.lock();
            auto base_time = _cloud_buf.front().first;
            auto cloud = _cloud_buf.front().second;
            _cloud_buf.pop_front();
            _cloud_buf_mtx.unlock();

            // Create new cloud
            CloudXYZIT corrected_cloud;
            size_t oldsize = corrected_cloud.points.size();
            size_t cloudsize = cloud.points.size();
            corrected_cloud.points.resize(oldsize + cloudsize);

            // Catch exception of no frames existing yet
            try
            {   
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
                    _tf_listener.waitForTransform("livox_frame", "world", time, ros::Duration(0.001));
                    _tf_listener.transformPoint("world", p_in, p_out);
                    corrected_cloud.points[oldsize + i].x = p_out.point.x;
                    corrected_cloud.points[oldsize + i].y = p_out.point.y;
                    corrected_cloud.points[oldsize + i].z = p_out.point.z;
                    corrected_cloud.points[oldsize + i].t = cloud.points[i].t;
                    corrected_cloud.points[oldsize + i].intensity = cloud.points[i].intensity; /// (0.04*pow((p_out.point.z-10), 2)+1);
                }

                // Publish corrected cloud
                sensor_msgs::PointCloud2 tempCloud;
                pcl::toROSMsg(corrected_cloud, tempCloud);
                tempCloud.header.stamp = ros::Time(base_time);
                tempCloud.header.frame_id = "world";
                _cloud_correct_pub.publish(tempCloud);
            }
            catch(...)
            {
                continue;
            }
            
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}

void LidarCorrectionNode::_dji_callback(const geometry_msgs::PointStamped::ConstPtr &PosMsg, const sensor_msgs::Imu::ConstPtr &ImuMsg)
{
    // Set rotation transform of drone from IMU message
    geometry_msgs::TransformStamped transform;
    transform.transform.rotation.w = ImuMsg->orientation.w;
    transform.transform.rotation.x = ImuMsg->orientation.x;
    transform.transform.rotation.y = ImuMsg->orientation.y;
    transform.transform.rotation.z = ImuMsg->orientation.z;

    // Zero position of drone by removing initial position
    static geometry_msgs::PointStamped first_point = *PosMsg;

    // Set translation transform of drone from POS message
    transform.transform.translation.x = PosMsg->point.x - first_point.point.x;
    transform.transform.translation.y = PosMsg->point.y - first_point.point.y;
    transform.transform.translation.z = PosMsg->point.z - first_point.point.z;

    //ROS_INFO("X: %f, Y: %f, Z: %f.", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);

    // Publish pose transform of drone
    transform.header.stamp = ImuMsg->header.stamp;
    transform.child_frame_id = "base_link";
    transform.header.frame_id = "world";
    _dynamic_broadcaster.sendTransform(transform);
}

void LidarCorrectionNode::_cloud_callback(const livox_ros_driver::CustomMsg::ConstPtr &msgIn)
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

void LidarCorrectionNode::_rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msgIn)
{
    std::pair <double, double> planar_coordinates = convert(msgIn->latitude, msgIn->longitude);
    ROS_INFO("Lat: %f, Lon: %f, x: %f, y: %f", msgIn->latitude, msgIn->longitude, planar_coordinates.first, 
            planar_coordinates.second);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Started Lidar Node!");

    LidarCorrectionNode node;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    node.node_thread();
    ros::waitForShutdown();
}
