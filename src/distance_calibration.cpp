#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Range.h>
#include <ros/console.h>
#include <iostream>

class RangeCalibration
{
    public:
        RangeCalibration()
        {
            nh_ = ros::NodeHandle("~");
            
            nh_.param<int>("maxSize", maxSize, 1000);

            node_namespace_ = ros::this_node::getNamespace();

            if (!node_namespace_.empty() && node_namespace_ != "/") 
            {
                subscribed_topic = node_namespace_ + "/" + subscribed_topic;
            }   


            // Subscriber for Range data
            range_sub1_ = nh_.subscribe("input_topic", 1, &RangeCalibration::rangeCallback1, this);

            // Publisher to republish the Range data
            range_pub1_ = nh_.advertise<sensor_msgs::Range>("output_topic", 10);

            range_offset = 0.0;

        }

        // Callback function for the first Range sensor
        void rangeCallback1(const sensor_msgs::Range::ConstPtr& msg) 
        {
            ROS_INFO_ONCE("Callback function for the first Range sensor.");
            sensor_msgs::Range range_data1_ = *msg;
            
            if (buffer1_.size() < maxSize)
            {
                ROS_INFO_ONCE("Buffering...");
                buffer1_.push_back(msg->range);
                return;
            }

			static bool initial_range = false;
			if (!initial_range)
            {
                ROS_INFO_ONCE("Calculate the average of the current elements in the buffer.");
			    initial_range = true;
                range_offset = calculateAverage(buffer1_);
                ROS_INFO_ONCE("Offset: %f", range_offset);
				return;
			}

            ROS_INFO_ONCE("Applying the offset.");
            // Apply the offset to the range data
            range_data1_.min_range = range_min;
            range_data1_.range = msg->range - range_offset + 5*range_min;

            range_pub1_.publish(range_data1_);
        }

        // Calculate the average of the current elements in the buffer
        double calculateAverage(std::deque<double> buffer) 
        {
            double range;
            double sum = 0.0;
            int index = 0;

            while (!buffer.empty()) 
            {
                range = buffer.front();
                sum += range;
                buffer.pop_front();
                index += 1;
            }
            return sum / index;
        }

    private:
        ros::NodeHandle nh_;
        std::string node_namespace_;
        ros::Subscriber range_sub1_;
        ros::Publisher range_pub1_;
        std::string subscribed_topic;
        std::string published_topic;
        std::deque<double> buffer1_;
        int maxSize;
        float range_min = 0.01;
        double range_offset;
};

int main(int argc, char** argv) {
    // Initialize ROS node with automatic node name based on namespace
    ros::init(argc, argv, "range_calibration_node");

    // Instantiate RangeCalibration class
    RangeCalibration calibrator;

    // Spin
    ros::spin();

    return 0;
}
