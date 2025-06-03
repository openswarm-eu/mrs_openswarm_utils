#include <ros/ros.h>
#include <tf/tf.h>
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
        nh.param<int>("maxSize", maxSize, 1000);
        nh.param<std::string>("uav_name", uav_name, "uav6");
        nh.param<bool>("imu_offset", imu_offset, false);
        nh.param<double>("wit_offset", wit_offset, 0.0);
        imu_a_sub_.subscribe(nh, "input_topic_main", 10);
        imu_b_sub_.subscribe(nh, "input_topic_ref", 10);

        sync_.reset(new Synchronizer<SyncPolicy>(SyncPolicy(10), imu_a_sub_, imu_b_sub_));
        sync_->registerCallback(boost::bind(&ImuMerger::callback, this, _1, _2));

        imu_pub_ = nh.advertise<Imu>("output_topic_imu", 10);
        orientation_pub_ = nh.advertise<geometry_msgs::QuaternionStamped>("output_topic_orientation", 1);
    }

    // Calculate the average of the current elements in the buffer
    double calculateAverage(std::deque<tf::Quaternion> buffer)
    {
        double roll, pitch, heading;
        double sum = 0.0;
        int index = 0;

        while (!buffer.empty())
        {
            tf::Matrix3x3(buffer.front()).getRPY(roll,pitch,heading);
            sum += heading;
            buffer.pop_front();
            index += 1;
        }
        return sum / index;

    }

    template<typename T>
    void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
    {
        double imuRoll, imuPitch, imuYaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

        *rosRoll = imuRoll;
        *rosPitch = imuPitch;
        *rosYaw = imuYaw;
    }

private:
    typedef sync_policies::ApproximateTime<Imu, Imu> SyncPolicy;

    ros::NodeHandle nh;
    message_filters::Subscriber<Imu> imu_a_sub_;
    message_filters::Subscriber<Imu> imu_b_sub_;
    boost::shared_ptr<Synchronizer<SyncPolicy>> sync_;
    ros::Publisher imu_pub_, orientation_pub_;
    std::string uav_name;
    double heading_imu, wit_offset;
    bool imu_offset;
    std::deque<tf::Quaternion> buffer2_;
    int maxSize;

    void callback(const ImuConstPtr& imu_a, const ImuConstPtr& imu_b)
    {
        ROS_INFO_ONCE("Callback function for the WIT IMU sensor.");
        sensor_msgs::Imu imu_data1_ = *imu_b;

        if (buffer2_.size() < maxSize)
        {
            ROS_INFO_ONCE("Buffering...");
            tf::Quaternion RQ2 = {imu_b->orientation.x,imu_b->orientation.y,imu_b->orientation.z,imu_b->orientation.w};
            buffer2_.push_back(RQ2);
            return;
        }

        static bool initial_heading = false;
        if (!initial_heading)
        {
            initial_heading = true;
            ROS_INFO_ONCE("Calculate the average of the current elements in the buffer.");
            heading_imu = calculateAverage(buffer2_);
            ROS_INFO_ONCE("Offset from WIT: %f", heading_imu);
            return;
        }

        Imu merged;
        geometry_msgs::QuaternionStamped q_msg;

        // Use header from IMU A
        merged.header = imu_a->header;
        merged.header.frame_id = uav_name + "/fcu";
        merged.header.stamp = ros::Time::now();

        // Orientation from IMU B
        merged.orientation = imu_b->orientation;
        merged.orientation_covariance = imu_b->orientation_covariance;

        double roll, pitch, heading;
        imuRPY2rosRPY(&imu_data1_, &roll, &pitch, &heading);
        if (imu_offset)
        {
            merged.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, heading - wit_offset);
        }

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
