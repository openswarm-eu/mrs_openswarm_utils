#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Float32.h>
#include <ros/console.h>
#include <iostream>

#include <tf/LinearMath/Quaternion.h>


class IMUCalibration
{
    public:
        IMUCalibration()
        {
            nh_ = ros::NodeHandle("~");
            nh_.param<int>("maxSize", maxSize, 1000);
            nh_.param<double>("offset", offset, 0.0);
            nh_.param<bool>("apply_offset", apply_offset, false);
            nh_.param<bool>("lla_offset", lla_offset, false);

            // Subscriber for IMU data
            imu_sub1_ = nh_.subscribe("input_topic", 1, &IMUCalibration::imuCallback1, this);

            // Subscriber for IMU data
            offset_sub_ = nh_.subscribe("input_topic_offset", 1, &IMUCalibration::offsetCallback, this);

            // Publisher to republish the IMU data
            imu_pub1_ = nh_.advertise<sensor_msgs::Imu>("output_topic_imu", 10);

            // Publisher to republish the IMU data
            orientation_pub1_ = nh_.advertise<geometry_msgs::QuaternionStamped>("output_topic_orientation", 10);

            heading_offset = 0.0;

        }

        // Callback function for the offset IMU sensor
        void offsetCallback(const std_msgs::Float32::ConstPtr& msg)
        {
            heading_offset = msg->data;
            ROS_INFO_ONCE("Offset: %f", heading_offset);
            received_offset = true;
        }

        // Callback function for the first IMU sensor
        void imuCallback1(const sensor_msgs::Imu::ConstPtr& msg)
        {
            ROS_INFO_ONCE("Callback function for the first IMU sensor.");
            sensor_msgs::Imu imu_data1_ = *msg;
            geometry_msgs::QuaternionStamped q_msg;

            if (!lla_offset)
            {
                if (buffer1_.size() < maxSize)
                {
                    ROS_INFO_ONCE("Buffering...");
                    tf::Quaternion RQ2 = {msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w};
                    buffer1_.push_back(RQ2);
                    return;
                }

                static bool initial_heading = false;
                if (!initial_heading)
                {
                    ROS_INFO_ONCE("Calculate the average of the current elements in the buffer.");
                    initial_heading = true;
                    heading_offset = calculateAverage(buffer1_);
                    return;
                }
            }
            else
            {
                if (!received_offset)
                {
                    ROS_INFO_ONCE("Waiting for the offset.");
                    return;
                }
            }

            if (apply_offset)
            {
                ROS_INFO_ONCE("Applying the offset.");
                ROS_INFO_ONCE("Offset: %f", heading_offset);
                double roll, pitch, heading;
                imuRPY2rosRPY(&imu_data1_, &roll, &pitch, &heading);
                imu_data1_.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, heading-(1.5707963 - heading_offset));
            }
            else
            {
                ROS_INFO_ONCE("No offset applied.");
            }

            imu_pub1_.publish(imu_data1_);
            q_msg.header = imu_data1_.header;
            q_msg.quaternion = imu_data1_.orientation;
            orientation_pub1_.publish(q_msg);
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
        ros::NodeHandle nh_;
        ros::Subscriber imu_sub1_, offset_sub_;
        ros::Publisher imu_pub1_, orientation_pub1_;
        std::string subscribed_topic;
        std::string published_topic;
        std::deque<tf::Quaternion> buffer1_;
        int maxSize;
        double heading_offset, offset;
        bool apply_offset, lla_offset;
        bool received_offset = false;
};



int main(int argc, char** argv) {
    // Initialize ROS node with automatic node name based on namespace
    ros::init(argc, argv, "imu_calibration_node");

    // Instantiate IMUCalibration class
    IMUCalibration calibrator;

    // Spin
    ros::spin();

    return 0;
}
