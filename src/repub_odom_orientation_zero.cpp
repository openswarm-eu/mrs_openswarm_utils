#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class OdomConv {
public:
    OdomConv() {
        nh_ = ros::NodeHandle("~");

        nh_.param<std::string>("subscribed_topic", subscribed_topic_, "odom_in");
        nh_.param<std::string>("published_topic", published_topic_, "odom_out");

        node_namespace_ = ros::this_node::getNamespace();

        if (!node_namespace_.empty() && node_namespace_ != "/") {
            subscribed_topic_ = node_namespace_ + "/" + subscribed_topic_;
        }

        odom_sub_ = nh_.subscribe(subscribed_topic_, 1, &OdomConv::odomCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(published_topic_, 1);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        nav_msgs::Odometry output_msg = *msg;

         output_msg.pose.pose.orientation.x = 0.0;
         output_msg.pose.pose.orientation.y = 0.0;
         output_msg.pose.pose.orientation.z = 0.0;
         output_msg.pose.pose.orientation.w = 1.0;

        output_msg.header.frame_id = node_namespace_.substr(1) + "/" + msg->header.frame_id;

        odom_pub_.publish(output_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;
    std::string subscribed_topic_;
    std::string published_topic_;
    std::string node_namespace_;
    double threshold_;
    int gps_status_;
};

int main(int argc, char** argv) {
    // Initialize ROS node with automatic node name based on namespace
    ros::init(argc, argv, "odom_conv");

    // Instantiate OdomConv class
    OdomConv odom_conv;

    // Spin
    ros::spin();

    return 0;
}
