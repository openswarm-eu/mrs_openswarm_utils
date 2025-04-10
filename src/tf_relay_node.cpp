#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

class TFRelayNode
{
public:
    TFRelayNode()
    {
        ros::NodeHandle nh;
        nh.param<std::string>("subscribed_topic", subscribed_topic_, "tf_throttle/tf");

        // Get the namespace of the node
        node_namespace_ = ros::this_node::getNamespace();

        if (!node_namespace_.empty() && node_namespace_ != "/") 
        {
            subscribed_topic_ = node_namespace_ + "/" + subscribed_topic_;
        }   

        tf_sub_ = nh.subscribe(subscribed_topic_, 100, &TFRelayNode::tfCallback, this);
    }

    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
    {
        for (const geometry_msgs::TransformStamped& transform : msg->transforms)
        {
            broadcaster_.sendTransform(transform);
        }
    }

private:
    ros::Subscriber tf_sub_;
    tf2_ros::TransformBroadcaster broadcaster_;
    std::string subscribed_topic_;
    std::string node_namespace_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_relay_node");
    TFRelayNode node;
    ros::spin();
    return 0;
}
