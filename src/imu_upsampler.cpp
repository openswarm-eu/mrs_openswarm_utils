#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

sensor_msgs::Imu last_imu, current_imu;
bool has_last = false;

double target_rate;

tf2::Quaternion mySlerp(const tf2::Quaternion& q0, const tf2::Quaternion& q1, double t) {
    return q0.slerp(q1, t);
}

sensor_msgs::Imu interpolateImu(const sensor_msgs::Imu& a,
                                const sensor_msgs::Imu& b,
                                ros::Time stamp)
{
    sensor_msgs::Imu out = a;

    double dt = (b.header.stamp - a.header.stamp).toSec();
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
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_upsampler");
    ros::NodeHandle nh;

    nh.param("target_rate", target_rate, 400.0); // output at 400 Hz

    ros::Subscriber sub = nh.subscribe("hw_api/imu_in", 10, imuCallback);
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("hw_api/imu", 50);

    ros::Rate r(target_rate);

    while (ros::ok())
    {
        ros::spinOnce();

        if (has_last)
        {
            ros::Time now = ros::Time::now();

            // Only interpolate within known window
            if (now > last_imu.header.stamp && now < current_imu.header.stamp) 
            {
                sensor_msgs::Imu out = interpolateImu(last_imu, current_imu, now);
                pub.publish(out);
            }
            // Otherwise just publish last known
            else
            {
                current_imu.header.stamp = now;
                pub.publish(current_imu);
            }
        }

        r.sleep();
    }

    return 0;
}
