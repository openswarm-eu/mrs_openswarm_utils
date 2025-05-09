#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geoid.hpp>
#include <deque>
#include <mutex>
#include <Eigen/Dense>

class GNSSOdom 
{
public:
    GNSSOdom() 
    {
        nh = ros::NodeHandle("~");
        gpsSub = nh.subscribe("input_topic", 1000, &GNSSOdom::GNSSCB, this,
                              ros::TransportHints().tcpNoDelay());
        getSub = nh.subscribe("input_topic_get_offset", 1, &GNSSOdom::getCommand, this);
        offsetPub = nh.advertise<std_msgs::Float32>("output_topic_offset", 10);
    }

private:
    void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) 
    {
        //std::cout << "gps status: " << msg->status.status << std::endl;
        if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) 
        {
            return;
        }
        Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
        //std::cout << "LLA: " << lla.transpose() << std::endl;
        if (!initENU) 
        {
            ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", lla[0], lla[1], lla[2]);
            geo_converter.Reset(lla[0], lla[1], lla[2]);
            initENU = true;
            return;
        }

        if (!get_lla) 
        {
            return;
        }

        double x, y, z;
        // LLA->ENU, better accuacy than gpsTools especially for z value
        geo_converter.Forward(lla[0], lla[1], lla[2], x, y, z);
        Eigen::Vector3d enu(x, y, z);
        if (abs(enu.x()) > 10000 || abs(enu.y()) > 10000 || abs(enu.z()) > 10000)
        {
            /** check your lla coordinate */
            ROS_INFO("Error origin : %f, %f, %f", enu(0), enu(1), enu(2));
            return;
        }

        double yaw = 0.0;
        std_msgs::Float32 offset_msg;
        double distance =
                sqrt(pow(enu(1) - prev_pose_left(1), 2) + pow(enu(0) - prev_pose_left(0), 2));
        if (distance > 0.1) 
        {
            yaw = atan2(enu(1) - prev_pose_left(1), enu(0) - prev_pose_left(0));
            yaw_quat_left = tf::createQuaternionMsgFromYaw(yaw);
            offset_msg.data = yaw;
            offsetPub.publish(offset_msg);
            prev_pose_left = enu;
            ROS_INFO("Yaw : %f", yaw);
            ROS_INFO("Cartesian : %f, %f, %f", enu(0), enu(1), enu(2));

        }
        get_lla = false;
    }

    void getCommand(const std_msgs::Empty::ConstPtr& msg) 
    {
        get_lla = true;
        ROS_INFO("Received an empty message.");
    }

    ros::NodeHandle nh;
    ros::Subscriber gpsSub, getSub;
    ros::Publisher offsetPub;
    bool initENU = false;
    bool get_lla = false;
    GeographicLib::LocalCartesian geo_converter;
    Eigen::Vector3d prev_pose_left;
    geometry_msgs::Quaternion yaw_quat_left;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "enu_orientation_corrector");
    GNSSOdom gps;
    ROS_INFO("\033[1;32m----> Simple GPS Odometry Started.\033[0m");
    ros::spin();
    return 1;
}