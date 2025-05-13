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
            nh_.param<bool>("apply_offset", apply_offset, false);
            nh_.param<bool>("self_offset", self_offset, false);
            nh_.param<bool>("imu_offset", imu_offset, false);
            nh_.param<bool>("lla_offset", lla_offset, false);

            // Subscriber for IMU data
            imu_sub1_ = nh_.subscribe("input_topic_main", 1, &IMUCalibration::imuCallback1, this);

            if (!apply_offset)
            {
                self_offset = false;
                imu_offset = false;
                lla_offset = false;
            }

            if (self_offset)
            {
                imu_offset = false;
                lla_offset = false;
            }

            // Subscriber for WIT IMU data
            if (imu_offset)
            {
                imu_sub2_ = nh_.subscribe("input_topic_ref", 1, &IMUCalibration::imuCallback2, this);
                lla_offset = false;
            }

            // Subscriber for LLA data
            if (lla_offset)
            {
                offset_sub_ = nh_.subscribe("input_topic_offset", 1, &IMUCalibration::offsetCallback, this);
            }

            // Publisher to republish the IMU data
            imu_pub1_ = nh_.advertise<sensor_msgs::Imu>("output_topic_imu", 10);

            // Publisher to republish the IMU data
            orientation_pub1_ = nh_.advertise<geometry_msgs::QuaternionStamped>("output_topic_orientation", 10);

            heading_offset = 0.0;

        }

        // Callback function for the offset IMU sensor from LLA
        void offsetCallback(const std_msgs::Float32::ConstPtr& msg)
        {
            ROS_INFO_ONCE("Callback function for LLA calibration.");
            heading_offset = msg->data;
            ROS_INFO_ONCE("Offset: %f", heading_offset);
            received_offset = true;
        }

        // Callback function for the first IMU sensor
        void imuCallback1(const sensor_msgs::Imu::ConstPtr& msg)
        {
            sensor_msgs::Imu imu_data1_ = *msg;
            geometry_msgs::QuaternionStamped q_msg;

            if (self_offset)
            {
                ROS_INFO_ONCE("Callback function for self calibration.");
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
                    ROS_INFO_ONCE("Offset: %f", heading_offset);
                    received_offset = true;
                    return;
                }
            }

            if (apply_offset)
            {
                if (!received_offset)
                {
                    ROS_INFO_ONCE("Waiting for the result.");
                    return;
                }
                
                ROS_INFO_ONCE("Applying the imu calibrationn.");
                double roll, pitch, heading;
                imuRPY2rosRPY(&imu_data1_, &roll, &pitch, &heading);
                if (imu_offset)
                {
                    imu_data1_.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, heading_imu);
                }
                else
                {
                    imu_data1_.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, heading - heading_offset);
                }
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

        // Callback function for the WIT sensor
        void imuCallback2(const sensor_msgs::Imu::ConstPtr& msg) 
        {
            ROS_INFO_ONCE("Callback function for the WIT IMU sensor.");
            sensor_msgs::Imu imu_data1_ = *msg;

            if (buffer2_.size() < maxSize)
            {
                ROS_INFO_ONCE("Buffering...");
                tf::Quaternion RQ2 = {msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w};
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
                received_offset = true;
                return;
            }
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
        ros::Subscriber imu_sub1_, imu_sub2_, offset_sub_;
        ros::Publisher imu_pub1_, orientation_pub1_;
        std::string subscribed_topic;
        std::string published_topic;
        std::deque<tf::Quaternion> buffer1_, buffer2_;
        int maxSize;
        double heading_imu, heading_offset;
        bool apply_offset, self_offset, imu_offset, lla_offset;
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
