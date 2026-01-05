#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <cmath>

class AdaptiveAltitudeFilter
{
public:
    AdaptiveAltitudeFilter()
    {
        ros::NodeHandle nh("~");

        nh.param("alpha_max", alpha_max_, 0.9);
        nh.param("alpha_min", alpha_min_, 0.05);
        nh.param("velocity_gain", velocity_gain_, 0.4);
        nh.param("drop_threshold", drop_threshold_, 0.5);

        lidar_sub_ = nh.subscribe("/hw_api/distance_sensor", 10,
                                  &AdaptiveAltitudeFilter::lidarCallback, this);

        odom_sub_ = nh.subscribe("/estimation_manager/odom_main", 10,
                                 &AdaptiveAltitudeFilter::odomCallback, this);

        alt_pub_ = nh.advertise<std_msgs::Float32>("/filtered_altitude", 10);

        initialized_ = false;
        v_xy_ = 0.0;
    }

private:
    ros::Subscriber lidar_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher alt_pub_;

    double alpha_max_;
    double alpha_min_;
    double velocity_gain_;
    double drop_threshold_;

    double filtered_altitude_;
    double v_xy_;

    bool initialized_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        const auto& v = msg->twist.twist.linear;
        v_xy_ = std::hypot(v.x, v.y);
        if (v_xy_ < 0.05)
            v_xy_ = 0.0;
    }

    void lidarCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
        double z_meas = msg->range;

        if (!initialized_)
        {
            filtered_altitude_ = z_meas;
            initialized_ = true;
        }
        else
        {
            // Reject sudden drops (bush protection)
            if (z_meas < filtered_altitude_ - drop_threshold_)
            {
                z_meas = filtered_altitude_;
            }

            double alpha = alpha_max_ - velocity_gain_ * v_xy_;
            if (alpha < alpha_min_)
                alpha = alpha_min_;
            else if (alpha > alpha_max_)
                alpha = alpha_max_;

            filtered_altitude_ =
                alpha * z_meas + (1.0 - alpha) * filtered_altitude_;
        }

        std_msgs::Float32 out;
        out.data = filtered_altitude_;
        alt_pub_.publish(out);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_altitude_filter");
    AdaptiveAltitudeFilter node;
    ros::spin();
    return 0;
}
