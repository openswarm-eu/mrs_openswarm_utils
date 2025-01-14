#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <ros/console.h>
#include <iostream>

#include <tf/LinearMath/Quaternion.h>


class IMUCalibration
{
    public:
        IMUCalibration()
        {
            nh_ = ros::NodeHandle("~");

            nh_.param<std::string>("subscribed_topic", subscribed_topic, "hw_api/imu");
            nh_.param<std::string>("published_topic", published_topic, "imu_out");
            nh_.param<int>("maxSize", maxSize, 1000);

            node_namespace_ = ros::this_node::getNamespace();

            if (!node_namespace_.empty() && node_namespace_ != "/") 
            {
                subscribed_topic = node_namespace_ + "/" + subscribed_topic;
            }   

            // Subscriber for IMU data
            imu_sub1_ = nh_.subscribe(subscribed_topic, 1, &IMUCalibration::imuCallback1, this);

            // Publisher to republish the IMU data
            imu_pub1_ = nh_.advertise<sensor_msgs::Imu>(published_topic, 1000);

            heading_offset = 0.0;

        }

        // Callback function for the first IMU sensor
        void imuCallback1(const sensor_msgs::Imu::ConstPtr& msg) 
        {
            ROS_INFO_ONCE("Callback function for the first IMU sensor.");
            sensor_msgs::Imu imu_data1_ = *msg;
            
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
				return;
			}

            ROS_INFO_ONCE("Applying the offset.");
            double roll, pitch, heading;
            imuRPY2rosRPY(&imu_data1_, &roll, &pitch, &heading);

            imu_data1_.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, heading-heading_offset);

            imu_pub1_.publish(imu_data1_);
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
        std::string node_namespace_;
        ros::Subscriber imu_sub1_;
        ros::Publisher imu_pub1_;
        std::string subscribed_topic;
        std::string published_topic;
        std::deque<tf::Quaternion> buffer1_;
        int maxSize;
        double heading_offset;
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
