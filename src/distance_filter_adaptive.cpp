#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
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
        nh.param("velocity_min", velocity_min_, 0.05);
        nh.param("velocity_max", velocity_max_, 5.0);
        nh.param("drop_threshold", drop_threshold_, 0.5);
        nh.param("bad_counter_limit", bad_counter_limit_, 4);

        lidar_sub_ = nh.subscribe("/hw_api/distance_sensor", 10,
                                  &AdaptiveAltitudeFilter::lidarCallback, this);

        vel_sub_ = nh.subscribe("/hw_api/velocity", 10,
                                 &AdaptiveAltitudeFilter::velCallback, this);

        alt_pub_ = nh.advertise<sensor_msgs::Range>("/filtered_altitude", 10);
        vel_pub_ = nh.advertise<std_msgs::Float32>("/velocity_xy", 10);
        alpha_pub_ = nh.advertise<std_msgs::Float32>("/filter_alpha", 10);

        initialized_ = false;
        bad_counter_ = 0;
        v_xy_ = 0.0;
    }

private:
    ros::Subscriber lidar_sub_;
    ros::Subscriber vel_sub_;
    ros::Publisher alt_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher alpha_pub_;

    double alpha_max_;
    double alpha_min_;
    double velocity_gain_;
    double velocity_min_;
    double velocity_max_;
    double drop_threshold_;

    double filtered_altitude_;
    double v_xy_;

    int bad_counter_;
    int bad_counter_limit_;

    bool initialized_;

    // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    // {
    //     const auto& v = msg->twist.twist.linear;
    //     v_xy_ = std::hypot(v.x, v.y);
    //     if (v_xy_ < 0.05)
    //         v_xy_ = 0.0;
    // }

    void velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        const auto& vx = msg->vector.x;
        const auto& vy = msg->vector.y;
        v_xy_ = std::hypot(vx, vy);
        if (v_xy_ < velocity_min_)
            v_xy_ = 0.0;
        std_msgs::Float32 out;
        out.data = v_xy_;
        vel_pub_.publish(out);
    }

    void lidarCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
        double z_meas = msg->range;

        if (!initialized_)
        {
            filtered_altitude_ = z_meas;
            initialized_ = true;
            return;
        }

        double innovation = z_meas - filtered_altitude_;
        bool suspicious = innovation < -drop_threshold_;

        if (suspicious)
            bad_counter_++;
        else
            bad_counter_ = 0;

        bool bush_detected = bad_counter_ >= bad_counter_limit_;

        double alpha;

        if (v_xy_ < velocity_min_)
        {
            // Takeoff / hover
            alpha = alpha_max_;
        }
        else if (bush_detected)
        {
            // Freeze altitude over bushes
            alpha = 0.0;
        }
        else
        {
            // Normal cruise
            alpha = alpha_min_;
        }

        filtered_altitude_ =
            alpha * z_meas + (1.0 - alpha) * filtered_altitude_;


        sensor_msgs::Range out_range;
        out_range.header = msg->header;
        out_range.radiation_type = msg->radiation_type;
        out_range.field_of_view = msg->field_of_view;
        out_range.min_range = msg->min_range;
        out_range.max_range = msg->max_range;
        out_range.range = filtered_altitude_;
        alt_pub_.publish(out_range);

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_altitude_filter");
    AdaptiveAltitudeFilter node;
    ros::spin();
    return 0;
}
