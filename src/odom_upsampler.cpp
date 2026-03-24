#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

nav_msgs::Odometry last_odom, current_odom;
bool has_last = false;

double target_rate;
std::string input_topic;
std::string output_topic;

tf2::Quaternion mySlerp(const tf2::Quaternion& q0, const tf2::Quaternion& q1, double t)
{
  return q0.slerp(q1, t);
}

nav_msgs::Odometry interpolateOdom(const nav_msgs::Odometry& a, const nav_msgs::Odometry& b, const ros::Time& stamp)
{
  nav_msgs::Odometry out = a;

  const double dt = (b.header.stamp - a.header.stamp).toSec();
  if (dt <= 0.0)
  {
    out = b;
    out.header.stamp = stamp;
    return out;
  }

  const double t = (stamp - a.header.stamp).toSec() / dt;

  out.header.frame_id = b.header.frame_id;
  out.child_frame_id = b.child_frame_id;

  // Pose position (linear interpolation)
  out.pose.pose.position.x = a.pose.pose.position.x + t * (b.pose.pose.position.x - a.pose.pose.position.x);
  out.pose.pose.position.y = a.pose.pose.position.y + t * (b.pose.pose.position.y - a.pose.pose.position.y);
  out.pose.pose.position.z = a.pose.pose.position.z + t * (b.pose.pose.position.z - a.pose.pose.position.z);

  // Pose orientation (SLERP)
  tf2::Quaternion qa, qb;
  tf2::fromMsg(a.pose.pose.orientation, qa);
  tf2::fromMsg(b.pose.pose.orientation, qb);
  tf2::Quaternion q_interp = mySlerp(qa, qb, t);
  q_interp.normalize();
  out.pose.pose.orientation = tf2::toMsg(q_interp);

  // Twist (linear interpolation)
  out.twist.twist.linear.x = a.twist.twist.linear.x + t * (b.twist.twist.linear.x - a.twist.twist.linear.x);
  out.twist.twist.linear.y = a.twist.twist.linear.y + t * (b.twist.twist.linear.y - a.twist.twist.linear.y);
  out.twist.twist.linear.z = a.twist.twist.linear.z + t * (b.twist.twist.linear.z - a.twist.twist.linear.z);

  out.twist.twist.angular.x = a.twist.twist.angular.x + t * (b.twist.twist.angular.x - a.twist.twist.angular.x);
  out.twist.twist.angular.y = a.twist.twist.angular.y + t * (b.twist.twist.angular.y - a.twist.twist.angular.y);
  out.twist.twist.angular.z = a.twist.twist.angular.z + t * (b.twist.twist.angular.z - a.twist.twist.angular.z);

  out.pose.covariance = b.pose.covariance;
  out.twist.covariance = b.twist.covariance;
  out.header.stamp = stamp;
  return out;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  last_odom = current_odom;
  current_odom = *msg;
  has_last = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_upsampler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("target_rate", target_rate, 100.0);
  pnh.param("input_topic", input_topic, std::string("odom_in"));
  pnh.param("output_topic", output_topic, std::string("odom"));

  if (target_rate <= 0.0)
  {
    ROS_WARN("Parameter ~target_rate must be > 0. Received %.3f, forcing to 100.0.", target_rate);
    target_rate = 100.0;
  }

  ros::Subscriber sub = nh.subscribe(input_topic, 10, odomCallback);
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>(output_topic, 50);

  ros::Rate rate(target_rate);

  while (ros::ok())
  {
    ros::spinOnce();

    if (has_last)
    {
      nav_msgs::Odometry out;
      const ros::Time now = ros::Time::now();

      // Only interpolate within known window.
      if (now > last_odom.header.stamp && now < current_odom.header.stamp)
      {
        out = interpolateOdom(last_odom, current_odom, now);
      }
      else
      {
        out = current_odom;
        out.header.stamp = now;
      }

      pub.publish(out);
    }

    rate.sleep();
  }

  return 0;
}
