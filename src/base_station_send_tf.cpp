#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <robot_localization/navsat_conversions.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

class GpsTransformPublisher
{
public:
    GpsTransformPublisher() : received_first_fix_(false)
    {
        nh_ = ros::NodeHandle("~");
        // Set gnss topic
        nh_.param<std::string>("subscribed_topic", subscribed_topic_, "hw_api/gnss");
        // Set base station GPS coordinates (fixed)
        nh_.param("base_station_lat", base_station_lat_, 47.397743);    // Latitude of the base station
        nh_.param("base_station_lon", base_station_lon_, 8.545594);     // Longitude of the base station
        // Set frames
        nh_.param<std::string>("frame_id", frame_id_, "base_station");
        nh_.param<std::string>("child_frame_id", child_frame_id_, "world");

        // Convert base station GPS coordinates to UTM
        RobotLocalization::NavsatConversions::UTM(base_station_lat_, base_station_lon_, &x0, &y0);

        // Subscriber for the robot's GPS data
        gps_sub_ = nh_.subscribe(subscribed_topic_, 10, &GpsTransformPublisher::gpsCallback, this);

        // Timer to continuously publish the transform at a constant frequency (10 Hz)
        publish_timer_ = nh_.createTimer(ros::Duration(0.1), &GpsTransformPublisher::publishTransform, this);
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if (!received_first_fix_)
        {
            if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
            {
                ROS_WARN("No GPS fix for the robot.");
                return;
            }

            // Convert robot's GPS (lat, lon) to UTM
            RobotLocalization::NavsatConversions::UTM(msg->latitude, msg->longitude, &x1, &y1);

            // Compute the relative position between the robot and the base station
            dx_ = x1 - x0;
            dy_ = y1 - y0;

            ROS_INFO("First GPS fix received. Publishing transform continuously...");

            received_first_fix_ = true; // Mark that we have received the first fix
        }
    }


    void publishTransform(const ros::TimerEvent&)
    {
        if (!received_first_fix_)
        {
            ROS_WARN("Waiting for the first GPS fix...");
            return;
        }
        tf::Transform base_station_to_world = tf::Transform(tf::createQuaternionFromRPY(
            0.0, 0.0, 0.0),
            tf::Vector3(dx_, dy_, 0.0));

        // // Publish the TF between the base station and the robot
        // geometry_msgs::TransformStamped transformStamped;
        // transformStamped.header.stamp = ros::Time::now();
        // transformStamped.header.frame_id = frame_id_;
        // transformStamped.child_frame_id = child_frame_id_;

        // transformStamped.transform.translation.x = dx_;
        // transformStamped.transform.translation.y = dy_;
        // transformStamped.transform.translation.z = 0.0;

        // // Assume no rotation between the base station and the robot
        // transformStamped.transform.rotation.x = 0.0;
        // transformStamped.transform.rotation.y = 0.0;
        // transformStamped.transform.rotation.z = 0.0;
        // transformStamped.transform.rotation.w = 1.0;

        //tf_broadcaster_.sendTransform(transformStamped.inverse());
        tf_broadcaster_.sendTransform(tf::StampedTransform(base_station_to_world.inverse(), ros::Time::now(), child_frame_id_, frame_id_));
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    // tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Timer publish_timer_;

    double base_station_lat_;
    double base_station_lon_;
    double x0, y0;
    double x1, y1;
    double dx_, dy_;

    std::string subscribed_topic_;
    std::string frame_id_;
    std::string child_frame_id_;

    bool received_first_fix_; // Flag to track if the first GPS fix has been received
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_transform_publisher");
    
    GpsTransformPublisher gps_transform_publisher;

    ros::spin();

    return 0;
}
