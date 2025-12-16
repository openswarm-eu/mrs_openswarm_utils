#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <deque>
#include <algorithm>

class RangeFilter
{
public:
    RangeFilter()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        pnh.param("median_window", median_window_, 5);
        pnh.param("ema_alpha", ema_alpha_, 0.2);
        pnh.param("max_jump_rate", max_jump_rate_, 1.5);
        pnh.param("max_range", max_range_, 10.0);
        pnh.param("bootstrap", bootstrap_, false);
        pnh.param("bootstrap_height", bootstrap_height_, 0.3);  // meters
        pnh.param("bootstrap_time", bootstrap_time_, 3.0);     // seconds
        pnh.param("jump_check", jump_check_, false);  // enable

        bootstrap_start_ = ros::Time::now();

        sub_ = nh.subscribe("hw_api/distance_sensor_in", 10, &RangeFilter::rangeCallback, this);
        pub_ = nh.advertise<sensor_msgs::Range>("hw_api/distance_sensor", 10);

        last_time_ = ros::Time(0);
        filtered_range_ = 0.0;
        
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::deque<float> window_;
    int median_window_;
    double ema_alpha_;
    double max_jump_rate_;
    double max_range_;

    bool bootstrap_, jump_check_;
    double bootstrap_height_;
    double bootstrap_time_;
    ros::Time bootstrap_start_;

    ros::Time last_time_;
    double filtered_range_;

    // float computeMedian()
    // {
    //     std::vector<float> temp(window_.begin(), window_.end());
    //     std::sort(temp.begin(), temp.end());
    //     return temp[temp.size() / 2];
    // }

    float computeMedian()
    {
        std::vector<float> temp;

        float max_allowed = filtered_range_ + 0.5f;  // adaptive ceiling rejection

        for (float v : window_)
        {
            if (v <= max_allowed)
            {
                temp.push_back(v);
            }
        }

        // SAFETY: if everything got rejected
        if (temp.empty())
        {
            return filtered_range_;  // fallback to last good value
        }

        std::sort(temp.begin(), temp.end());
        return temp[temp.size() / 2];
    }


    void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
        // Reject invalid readings
        if (!std::isfinite(msg->range))
            return;

        float raw = msg->range;

        // Clamp absurd values (ceiling issue)
        raw = std::min(raw, static_cast<float>(max_range_));

        // Fill median window
        window_.push_back(raw);
        if (window_.size() > median_window_)
            window_.pop_front();

        float median = computeMedian();

        // Time delta
        ros::Time now = msg->header.stamp;
        if (last_time_.isZero())
        {
            filtered_range_ = median;
            last_time_ = now;
            publish(msg, filtered_range_);
            return;
        }

        double dt = (now - last_time_).toSec();
        last_time_ = now;

        // Bootstrap logic
        if (bootstrap_)
        {
            ROS_INFO_ONCE("Range filter: entering bootstrap phase");
            filtered_range_ = median;

            // Exit conditions
            if (filtered_range_ > bootstrap_height_ ||
                (ros::Time::now() - bootstrap_start_).toSec() > bootstrap_time_)
            {
                bootstrap_ = false;
                ROS_INFO("Range filter: exiting bootstrap phase");
            }

            publish(msg, filtered_range_);
            return;
        }

        // JUMP CHECK (only for increasing distance)
        if (jump_check_)
        {
            double max_jump = max_jump_rate_ * dt;
            double diff = fabs(median - filtered_range_);
            ROS_INFO_THROTTLE(1.0, "Range filter: dt=%.3f, max_allowed_jump=%.3f, diff=%.3f", dt, max_jump, diff);

            if (diff > max_jump)
            {
                publish(msg, filtered_range_);
                return;
            }
        }

        // Exponential moving average
        filtered_range_ =
            ema_alpha_ * median + (1.0 - ema_alpha_) * filtered_range_;

        publish(msg, filtered_range_);
    }

    void publish(const sensor_msgs::Range::ConstPtr& in, double value)
    {
        sensor_msgs::Range out = *in;
        out.range = value;
        pub_.publish(out);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_filter_node");
    RangeFilter rf;
    ros::spin();
    return 0;
}
