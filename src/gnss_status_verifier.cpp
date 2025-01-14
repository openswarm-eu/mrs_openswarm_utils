/// STATUS_FIX (https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatStatus.html)
// int8 STATUS_NO_FIX =  -1        # unable to fix position
// int8 STATUS_FIX =      0        # unaugmented fix
// int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
// int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

/// BESTPOS Position Type (https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm)
// CTU (https://ctu-mrs.github.io/mrs_modules_msgs/msg/Bestpos.html)
// 16 - SINGLE | Solution calculated using only data supplied by the GNSS satellites
// 32 - L1_FLOAT | Single-frequency RTK solution with unresolved, float carrier phase ambiguities
// 48 - L1_INT | Single-frequency RTK solution with carrier phase ambiguities resolved to integers


#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mrs_modules_msgs/Bestpos.h>


class GNSSVerifier {
public:
    GNSSVerifier() {
        nh_ = ros::NodeHandle("~");

        nh_.param<std::string>("subscribed_topic", subscribed_topic_, "hw_api/gnss");
        nh_.param<std::string>("published_topic", published_topic_, "gnss_out");
        nh_.param<std::string>("bestpos_topic", bestpos_topic_, "rtk/bestpos");
        nh_.param("use_bestpos", use_bestpos_, false);
        nh_.param("threshold", threshold_, 0.003);

        node_namespace_ = ros::this_node::getNamespace();

        if (!node_namespace_.empty() && node_namespace_ != "/") {
            subscribed_topic_ = node_namespace_ + "/" + subscribed_topic_;
            bestpos_topic_ = node_namespace_ + "/" + bestpos_topic_;
        }

        gnss_sub_ = nh_.subscribe(subscribed_topic_, 1, &GNSSVerifier::gnssCallback, this);
        bestpos_sub_ = nh_.subscribe(bestpos_topic_, 1, &GNSSVerifier::bestposCallback, this);
        gnss_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(published_topic_, 1);

        temp_msg.header.stamp = ros::Time(0);
    }

    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        output_msg = *msg;

            // Verify position_covariance
            bool under_threshold = true;
            for (auto cov : msg->position_covariance) {
                if (cov > threshold_) {
                    under_threshold = false;
                    break;
                }
            }

            // Modify status field accordingly
            if (under_threshold) {
                output_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            } else {
                output_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            }

        output_msg.header.frame_id = node_namespace_.substr(1) + "/" + "fcu";

        if (!use_bestpos_){
            gnss_pub_.publish(output_msg);
            ROS_INFO_THROTTLE(10, "Info from hw_api/gnss");
        }
    }

    void bestposCallback(const mrs_modules_msgs::Bestpos::ConstPtr& msg) {

        // Check position_type
         if (msg->position_type == "L1_INT") {
            temp_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            
        } else {
            temp_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }
        
        temp_msg.header.stamp = ros::Time::now();
        output_msg.status.status = temp_msg.status.status;
        output_msg.header.stamp = temp_msg.header.stamp;


        if (use_bestpos_){
            gnss_pub_.publish(output_msg);
            ROS_INFO_THROTTLE(10, "Info from rtk/bestpos");
        }
    }    

private:
    ros::NodeHandle nh_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber bestpos_sub_;
    ros::Publisher gnss_pub_;
    std::string subscribed_topic_;
    std::string bestpos_topic_;
    std::string published_topic_;
    std::string node_namespace_;
    sensor_msgs::NavSatFix temp_msg;
    sensor_msgs::NavSatFix output_msg;
    double threshold_;
    bool use_bestpos_;
};

int main(int argc, char** argv) {
    // Initialize ROS node with automatic node name based on namespace
    ros::init(argc, argv, "gnss_verifier");

    // Instantiate GNSSVerifier class
    GNSSVerifier verifier;

    // Spin
    ros::spin();

    return 0;
}
