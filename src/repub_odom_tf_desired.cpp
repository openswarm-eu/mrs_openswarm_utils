#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OdomPoseRepublisher
{
public:
    OdomPoseRepublisher()
        : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        private_nh.param<std::string>("target_frame", target_frame_, "base_link");

        odom_sub_ = nh.subscribe("odom", 10, &OdomPoseRepublisher::odomCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_transformed", 10);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {

        if (!tf_buffer_.canTransform(target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.5)))
        {
            ROS_WARN_THROTTLE(1.0, "Transform from %s to %s not available yet.",
                        msg->header.frame_id.c_str(), target_frame_.c_str());
            return;
        }
        try
        {

            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.5));

            nav_msgs::Odometry odom_transformed;
            odom_transformed.header.stamp = msg->header.stamp;
            odom_transformed.header.frame_id = msg->header.frame_id;

            // Transform the pose only
            tf2::doTransform(msg->pose.pose, odom_transformed.pose.pose, transform);

            // Copy velocity data directly
            odom_transformed.twist = msg->twist;
            odom_transformed.child_frame_id = msg->child_frame_id;

            odom_pub_.publish(odom_transformed);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "Transform error: %s", ex.what());
        }
    }

private:
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string target_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_pose_republisher");
    OdomPoseRepublisher republisher;
    ros::spin();
    return 0;
}
