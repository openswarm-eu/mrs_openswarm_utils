#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace sensor_msgs;
using namespace message_filters;

class ImuMerger
{
public:
    ImuMerger()
    {
        nh = ros::NodeHandle("~");
        nh.param<std::string>("uav_name", uav_name, "uav6");
        imu_a_sub_.subscribe(nh, "input_topic_main", 10);
        imu_b_sub_.subscribe(nh, "input_topic_ref", 10);

        sync_.reset(new Synchronizer<SyncPolicy>(SyncPolicy(10), imu_a_sub_, imu_b_sub_));
        sync_->registerCallback(boost::bind(&ImuMerger::callback, this, _1, _2));

        imu_pub_ = nh.advertise<Imu>("output_topic_imu", 10);
        orientation_pub_ = nh.advertise<geometry_msgs::QuaternionStamped>("output_topic_orientation", 1);
    }

private:
    typedef sync_policies::ApproximateTime<Imu, Imu> SyncPolicy;

    ros::NodeHandle nh;
    message_filters::Subscriber<Imu> imu_a_sub_;
    message_filters::Subscriber<Imu> imu_b_sub_;
    boost::shared_ptr<Synchronizer<SyncPolicy>> sync_;
    ros::Publisher imu_pub_, orientation_pub_;
    std::string uav_name;

    void callback(const ImuConstPtr& imu_a, const ImuConstPtr& imu_b)
    {
        Imu merged;
        geometry_msgs::QuaternionStamped q_msg;

        // Use header from IMU A
        merged.header = imu_a->header;
        merged.header.frame_id = uav_name + "/fcu";
        merged.header.stamp = ros::Time::now();

        // Orientation from IMU B
        merged.orientation = imu_b->orientation;
        merged.orientation_covariance = imu_b->orientation_covariance;

        // Angular velocity and acceleration from IMU A
        merged.angular_velocity = imu_b->angular_velocity;
        merged.angular_velocity_covariance = imu_b->angular_velocity_covariance;

        // merged.linear_acceleration.x = 0.0;
        // merged.linear_acceleration.y = 0.0;
        // merged.linear_acceleration.z = imu_b->linear_acceleration.z;
        merged.linear_acceleration = imu_b->linear_acceleration;
        merged.linear_acceleration_covariance = imu_b->linear_acceleration_covariance;

        imu_pub_.publish(merged);
        q_msg.header = merged.header;
        q_msg.quaternion = merged.orientation;
        orientation_pub_.publish(q_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_merger_node");

    ImuMerger merger;

    ros::spin();
    return 0;
}
