#include <mutex>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

class OdomAltitudeOffset
{
public:
  OdomAltitudeOffset()
  {
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("odom1_topic", odom1_topic_, "odom1");
    pnh.param<std::string>("odom2_topic", odom2_topic_, "odom2");
    pnh.param<std::string>("output_topic", output_topic_, "odom2_offset");
    pnh.param<std::string>("service_name", service_name_, "capture_altitude");

    odom1_sub_ = nh_.subscribe(odom1_topic_, 10, &OdomAltitudeOffset::odom1Callback, this);
    odom2_sub_ = nh_.subscribe(odom2_topic_, 50, &OdomAltitudeOffset::odom2Callback, this);
    odom2_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 50);

    service_ = pnh.advertiseService(service_name_, &OdomAltitudeOffset::captureAltitudeCb, this);

    ROS_INFO(
      "odom_altitude_offset ready. odom1='%s' odom2='%s' output='%s' service='~%s'",
      odom1_topic_.c_str(), odom2_topic_.c_str(), output_topic_.c_str(), service_name_.c_str());
  }

private:
  void odom1Callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    latest_odom1_z_ = msg->pose.pose.position.z;
    has_odom1_ = true;
  }

  void odom2Callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    nav_msgs::Odometry out = *msg;
    {
      std::lock_guard<std::mutex> lock(data_mtx_);
      latest_odom2_z_ = msg->pose.pose.position.z;
      has_odom2_ = true;

      if (calibrated_)
      {
        out.pose.pose.position.z = msg->pose.pose.position.z + offset_;
      }
    }
    odom2_pub_.publish(out);
  }

  bool captureAltitudeCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);

    if (calibrated_)
    {
      res.success = false;
      res.message = "Calibration already executed once.";
      return true;
    }

    if (!has_odom1_ || !has_odom2_)
    {
      res.success = false;
      res.message = "Waiting for odom1/odom2 messages before calibration.";
      return true;
    }

    altitude_value_ = latest_odom1_z_;
    offset_ = altitude_value_ - latest_odom2_z_;
    calibrated_ = true;

    res.success = true;
    res.message =
      "Captured altitude_value and computed offset. altitude_value=" + std::to_string(altitude_value_) +
      " offset=" + std::to_string(offset_);

    ROS_INFO("Calibration done once: altitude_value=%.6f latest_odom2_z=%.6f offset=%.6f",
             altitude_value_, latest_odom2_z_, offset_);
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom1_sub_;
  ros::Subscriber odom2_sub_;
  ros::Publisher odom2_pub_;
  ros::ServiceServer service_;

  std::string odom1_topic_;
  std::string odom2_topic_;
  std::string output_topic_;
  std::string service_name_;

  std::mutex data_mtx_;
  bool has_odom1_ = false;
  bool has_odom2_ = false;
  bool calibrated_ = false;

  double latest_odom1_z_ = 0.0;
  double latest_odom2_z_ = 0.0;
  double altitude_value_ = 0.0;
  double offset_ = 0.0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_altitude_offset");
  OdomAltitudeOffset node;
  ros::spin();
  return 0;
}
