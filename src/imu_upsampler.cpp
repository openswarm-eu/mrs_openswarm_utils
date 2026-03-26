#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <deque>
#include <cmath>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

sensor_msgs::Imu last_imu, current_imu;
bool has_last = false;

double target_rate;
bool apply_offset_enabled = false;
bool external_imu = false;
int max_size = 1000;
bool heading_offset_ready = false;
double heading_offset = 0.0;
std::deque<double> yaw_buffer;
constexpr double kRadToDeg = 57.29577951308232;
ros::Publisher orientation_pub;
std::string orientation_topic_in;
std::string orientation_topic_out;

tf2::Quaternion mySlerp(const tf2::Quaternion& q0, const tf2::Quaternion& q1, double t) {
    return q0.slerp(q1, t);
}

double quaternionToYaw(const geometry_msgs::Quaternion& q_msg)
{
    tf2::Quaternion q;
    tf2::fromMsg(q_msg, q);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

double computeAverageYaw(const std::deque<double>& yaws)
{
    double sum_sin = 0.0;
    double sum_cos = 0.0;

    for (const auto& yaw : yaws)
    {
        sum_sin += std::sin(yaw);
        sum_cos += std::cos(yaw);
    }

    return std::atan2(sum_sin, sum_cos);
}

void applyHeadingOffset(geometry_msgs::Quaternion& q_msg)
{
    tf2::Quaternion q;
    tf2::fromMsg(q_msg, q);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw -= heading_offset;

    tf2::Quaternion q_corrected;
    q_corrected.setRPY(roll, pitch, yaw);
    q_corrected.normalize();
    q_msg = tf2::toMsg(q_corrected);
}

void applyHeadingOffset(sensor_msgs::Imu& imu_msg)
{
    applyHeadingOffset(imu_msg.orientation);
}

void orientationCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    if (!orientation_pub)
    {
        return;
    }

    if (apply_offset_enabled && !heading_offset_ready)
    {
        ROS_WARN_THROTTLE(2.0, "Waiting for heading self-calibration before publishing orientation topic.");
        return;
    }

    geometry_msgs::QuaternionStamped out = *msg;
    if (apply_offset_enabled)
    {
        applyHeadingOffset(out.quaternion);
    }
    orientation_pub.publish(out);
}

void publishOrientationFromImu(const sensor_msgs::Imu& imu_msg)
{
    if (!orientation_pub)
    {
        return;
    }

    geometry_msgs::QuaternionStamped out;
    out.header = imu_msg.header;
    out.quaternion = imu_msg.orientation;
    orientation_pub.publish(out);
}

sensor_msgs::Imu interpolateImu(const sensor_msgs::Imu& a,
                                const sensor_msgs::Imu& b,
                                ros::Time stamp)
{
    sensor_msgs::Imu out = a;

    double dt = (b.header.stamp - a.header.stamp).toSec();
    if (dt <= 0.0)
    {
        out = b;
        out.header.stamp = stamp;
        return out;
    }

    double t  = (stamp - a.header.stamp).toSec() / dt;

    // Orientation SLERP
    tf2::Quaternion qa, qb;
    tf2::fromMsg(a.orientation, qa);
    tf2::fromMsg(b.orientation, qb);
    tf2::Quaternion q_interp = mySlerp(qa, qb, t);
    out.orientation = tf2::toMsg(q_interp);

    // Angular velocity (linear interpolation)
    out.angular_velocity.x = a.angular_velocity.x + t * (b.angular_velocity.x - a.angular_velocity.x);
    out.angular_velocity.y = a.angular_velocity.y + t * (b.angular_velocity.y - a.angular_velocity.y);
    out.angular_velocity.z = a.angular_velocity.z + t * (b.angular_velocity.z - a.angular_velocity.z);

    // Linear acceleration (linear interpolation)
    out.linear_acceleration.x = a.linear_acceleration.x + t * (b.linear_acceleration.x - a.linear_acceleration.x);
    out.linear_acceleration.y = a.linear_acceleration.y + t * (b.linear_acceleration.y - a.linear_acceleration.y);
    out.linear_acceleration.z = a.linear_acceleration.z + t * (b.linear_acceleration.z - a.linear_acceleration.z);

    out.header.stamp = stamp;
    return out;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    last_imu = current_imu;
    current_imu = *msg;
    has_last = true;

    if (apply_offset_enabled && !heading_offset_ready)
    {
        yaw_buffer.push_back(quaternionToYaw(msg->orientation));

        if (static_cast<int>(yaw_buffer.size()) >= max_size)
        {
            heading_offset = computeAverageYaw(yaw_buffer);
            heading_offset_ready = true;
            ROS_INFO("Heading self-calibration completed. Offset yaw: %.6f rad (%.3f deg) using %d samples.",
                     heading_offset, heading_offset * kRadToDeg, max_size);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_upsampler");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("target_rate", target_rate, 400.0); // output at 400 Hz
    pnh.param("apply_offset_enabled", apply_offset_enabled, false);
    pnh.param("external_imu", external_imu, false);
    pnh.param("max_size", max_size, 1000);
    pnh.param("orientation_topic_in", orientation_topic_in, std::string("hw_api/orientation_in"));
    pnh.param("orientation_topic_out", orientation_topic_out, std::string("hw_api/orientation"));

    if (max_size <= 0)
    {
        ROS_WARN("Parameter ~max_size must be > 0. Received %d, forcing to 1.", max_size);
        max_size = 1;
    }

    ros::Subscriber sub = nh.subscribe("hw_api/imu_in", 10, imuCallback);
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("hw_api/imu", 50);
    ros::Subscriber orientation_sub;
    if (!external_imu)
    {
        orientation_sub = nh.subscribe(orientation_topic_in, 10, orientationCallback);
    }
    orientation_pub = nh.advertise<geometry_msgs::QuaternionStamped>(orientation_topic_out, 50);

    ros::Rate r(target_rate);

    while (ros::ok())
    {
        ros::spinOnce();

        if (has_last)
        {
            if (apply_offset_enabled && !heading_offset_ready)
            {
                ROS_WARN_THROTTLE(2.0, "Waiting for heading self-calibration: %zu/%d samples.",
                                  yaw_buffer.size(), max_size);
                r.sleep();
                continue;
            }

            ros::Time now = ros::Time::now();

            // Only interpolate within known window
            if (now > last_imu.header.stamp && now < current_imu.header.stamp) 
            {
                sensor_msgs::Imu out = interpolateImu(last_imu, current_imu, now);
                if (apply_offset_enabled)
                {
                    applyHeadingOffset(out);
                }
                pub.publish(out);
                if (external_imu)
                {
                    publishOrientationFromImu(out);
                }
            }
            // Otherwise just publish last known
            else
            {
                sensor_msgs::Imu out = current_imu;
                out.header.stamp = now;
                if (apply_offset_enabled)
                {
                    applyHeadingOffset(out);
                }
                pub.publish(out);
                if (external_imu)
                {
                    publishOrientationFromImu(out);
                }
            }
        }

        r.sleep();
    }

    return 0;
}
