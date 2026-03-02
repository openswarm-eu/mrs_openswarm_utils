#include <mutex>
#include <string>
#include <cmath>
#include <limits>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "mrs_openswarm_utils/save_map.h"

class GlobalMapProvider
{
public:
  GlobalMapProvider()
  {
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("global_map_topic", global_map_topic_, "global_map");
    pnh.param<bool>("allow_empty_response", allow_empty_response_, false);

    map_sub_ = nh_.subscribe(global_map_topic_, 1, &GlobalMapProvider::mapCallback, this);
    get_map_srv_ = pnh.advertiseService("get_global_map", &GlobalMapProvider::getMapCallback, this);

    ROS_INFO("GlobalMapProvider ready: topic='%s', service='~get_global_map'", global_map_topic_.c_str());
  }

private:
  void mapCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = *msg;
    has_map_ = true;
  }

  bool getMapCallback(mrs_openswarm_utils::save_map::Request& req, mrs_openswarm_utils::save_map::Response& res)
  {
    sensor_msgs::PointCloud2 local_copy;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!has_map_)
      {
        if (allow_empty_response_)
        {
          ROS_WARN_THROTTLE(2.0, "No global map received yet. Returning empty map.");
          res.cloud = sensor_msgs::PointCloud2();
          return true;
        }

        ROS_WARN_THROTTLE(2.0, "No global map received yet. Service call failed.");
        return false;
      }
      local_copy = latest_map_;
    }

    // "Empty" for float fields is represented as 0.0 in ROS service requests.
    // Keep original point cloud resolution when resolution is empty/invalid.
    const bool keep_original_resolution =
      (!std::isfinite(req.resolution)) ||
      (req.resolution <= std::numeric_limits<float>::epsilon());

    if (!keep_original_resolution)
    {
      pcl::PointCloud<pcl::PointXYZ> input_cloud;
      pcl::fromROSMsg(local_copy, input_cloud);

      pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setLeafSize(req.resolution, req.resolution, req.resolution);
      voxel.setInputCloud(input_cloud.makeShared());
      voxel.filter(filtered_cloud);

      pcl::toROSMsg(filtered_cloud, res.cloud);
      res.cloud.header = local_copy.header;
    }
    else
    {
      res.cloud = local_copy;
    }

    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::ServiceServer get_map_srv_;

  std::string global_map_topic_;
  bool allow_empty_response_ = false;

  std::mutex map_mutex_;
  sensor_msgs::PointCloud2 latest_map_;
  bool has_map_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_map_provider");
  GlobalMapProvider provider;
  ros::spin();
  return 0;
}
