#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include "mrs_openswarm_utils/save_map.h"
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <vector>
#include <string>
#include <thread>
#include <future>
#include <chrono>
#include <boost/shared_ptr.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Define point cloud types
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class MapGeneration
{
    public:
     
        MapGeneration()
        {
            nh_ = ros::NodeHandle("~");

            std::string ns = nh_.getNamespace();
            if (!ns.empty() && ns.front() == '/')
            {
                ns.erase(0, 1);
            }
            const std::size_t slash_pos = ns.find('/');
            uav_name_ = (slash_pos == std::string::npos) ? ns : ns.substr(0, slash_pos);

            nh_.param<std::vector<std::string>>("uav_names", uav_names_, std::vector<std::string>());
            nh_.param<std::string>("frame_output", frame_output_, "common_origin");
            nh_.param<std::string>("global_map_service", global_map_service_, "global_map_provider/get_global_map");
            nh_.param("wait_time", wait_time_, 5.0);
            nh_.param("save_pcd_file", save_pcd_file_, true);
            nh_.param("use_icp", use_icp_, true);

            ROS_INFO("Name of UAV:  %s", uav_name_.c_str());
            ROS_INFO("Number of Robots:  %li", uav_names_.size());

            // Publisher global map
            map_pub = nh_.advertise<sensor_msgs::PointCloud2>("globalMap", 1);
            // Service Server
            srvSaveMap  = nh_.advertiseService("save_map", &MapGeneration::saveMapService, this);

            if (uav_names_.empty())
            {
                ROS_WARN("Parameter 'uav_names' is empty. save_map service will not be able to aggregate clouds.");
            }

            for(int it = 0; it < uav_names_.size(); it++)
            {
                singleRobot robot;
                std::string robot_name_list = uav_names_[it].c_str();
                name_ = "/" + robot_name_list;

                robot.id_ = it; // robot ID and name
                robot.name_ = name_;

                const std::string service_suffix =
                  (!global_map_service_.empty() && global_map_service_.front() == '/') ? global_map_service_ : "/" + global_map_service_;
                robot.service_name_ = name_ + service_suffix;
                robot.srv_client_map = nh_.serviceClient<mrs_openswarm_utils::save_map>(robot.service_name_);

                robots.push_back(robot);
                save_it.push_back(it);
            }

            // Leader reference is always the first UAV in uav_names.
            robot_target = 0;

            ROS_INFO_ONCE("Map Generation Service is ready.");
        }

        // Callback function for the first drone
        void mapHandler(const sensor_msgs::PointCloud2ConstPtr& msg, int& id)
        {
            pcl::fromROSMsg(*msg, *robots[id].cloud);
            robots[id].cloud_received = true;
            ROS_INFO("Received cloud:  %d", id);
        }

        bool saveMapService(mrs_openswarm_utils::save_mapRequest& req, mrs_openswarm_utils::save_mapResponse& res)
        {

            // Path to save map: TODO
            const char* pwd = std::getenv("PWD");
            if(req.destination.empty()) saveMapDirectory_ = (pwd != nullptr) ? pwd : ".";
            else saveMapDirectory_ = ((pwd != nullptr) ? std::string(pwd) : std::string(".")) + req.destination;
            ROS_INFO("Save destination:  %s", saveMapDirectory_.c_str());

            // Wait for the service to be available
            ros::Duration timeout_(wait_time_);

            size_t successful_maps = 0;
            
            for(int it = 0; it < uav_names_.size(); it++)
            {
                robots[it].cloud_received = false;
                robots[it].cloud->clear();

                // Wait for the service with a timeout
                if (ros::service::waitForService(robots[it].service_name_, timeout_))
                {
                    auto service_request = std::make_shared<mrs_openswarm_utils::save_map>();
                    service_request->request.resolution = req.resolution;
                    service_request->request.destination = "";

                    std::shared_ptr<std::promise<bool>> prom_ptr = std::make_shared<std::promise<bool>>();
                    std::future<bool> fut = prom_ptr->get_future();
                    ros::ServiceClient service_client = robots[it].srv_client_map;

                    std::thread(
                      [service_client, service_request, prom_ptr]() mutable {
                        bool ok = false;
                        try
                        {
                          ok = service_client.call(*service_request);
                        }
                        catch (...)
                        {
                          ok = false;
                        }
                        prom_ptr->set_value(ok);
                      })
                      .detach();

                    const auto wait_status = fut.wait_for(std::chrono::duration<double>(wait_time_));
                    if (wait_status != std::future_status::ready)
                    {
                        ROS_ERROR("Timeout: map service call to %s exceeded %.2f s.", robots[it].name_.c_str(), wait_time_);
                        continue;
                    }

                    const bool call_ok = fut.get();
                    if (call_ok)
                    {
                        sensor_msgs::PointCloud2 point_cloud = service_request->response.cloud;
                        pcl::fromROSMsg(point_cloud, *robots[it].cloud);
                        if (!robots[it].cloud->empty())
                        {
                            robots[it].cloud_received = true;
                            successful_maps++;
                        }
                        else
                        {
                            ROS_WARN("Received empty map from %s.", robots[it].name_.c_str());
                        }

                        // Print some details about the point cloud (for demonstration)
                        ROS_INFO("Map service call successfull: %s", robots[it].name_.c_str());
                        ROS_INFO("Received point cloud with width: %d, height: %d", point_cloud.width, point_cloud.height);
                        ROS_INFO("Frame ID: %s", point_cloud.header.frame_id.c_str());
                    }
                    else
                    {
                        ROS_ERROR("Failed to call map service: %s", robots[it].name_.c_str());
                    }
                }
                else
                {
                    ROS_ERROR("Timeout: Service %s is not available after waiting.", robots[it].name_.c_str());
                }
            }

            if (successful_maps == 0)
            {
                ROS_ERROR("No valid maps were received from any UAV.");
                return false;
            }

            // Combine the point clouds
            PointCloud::Ptr combined_cloud(new PointCloud);

            const bool leader_available =
              (robot_target >= 0) &&
              (robot_target < static_cast<int>(robots.size())) &&
              robots[robot_target].cloud_received;

            if (use_icp_ == true && leader_available)
            {
                // **************** ICP Module n UAVs ****************
                // Create ICP object
                pcl::IterativeClosestPoint<PointT, PointT> icp;
                ROS_INFO("ICP will be used.");

                for(int it = 0; it < uav_names_.size(); it++)
                {
                    if (robot_target == it)
                    {
                        ROS_INFO("ICP target: %d.", robot_target);
                        icp.setInputTarget(robots[it].cloud);   // Reference cloud
                    }
                }

                for(int it = 0; it < uav_names_.size(); it++)
                {
                    ROS_INFO("ICP source: %s.", robots[it].name_.c_str());
                    if (!robots[it].cloud_received)
                    {
                        ROS_WARN("Skipping %s in ICP merge: no valid cloud received.", robots[it].name_.c_str());
                        continue;
                    }

                    if (robot_target == it)
                    {
                        *combined_cloud = *combined_cloud + *robots[it].cloud;
                        continue;
                    }

                    // Set input clouds
                    icp.setInputSource(robots[it].cloud);   // Cloud to be aligned

                    // Set ICP parameters (optional tuning)
                    icp.setMaxCorrespondenceDistance(0.1); // Maximum allowed distance between points
                    icp.setTransformationEpsilon(1e-8);     // Convergence criteria
                    icp.setMaximumIterations(50);           // Maximum iterations

                    // Output point cloud
                    PointCloudT::Ptr final_cloud(new PointCloudT);

                    // Perform ICP alignment
                    icp.align(*final_cloud);

                    // Check if the ICP converged
                    if (icp.hasConverged()) {
                        std::cout << "ICP converged." << std::endl;
                        std::cout << "Score: " << icp.getFitnessScore() << std::endl;
                        Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
                        std::cout << "Transformation matrix: \n" << transformation_matrix << std::endl;
                        *combined_cloud = *combined_cloud + *final_cloud;
                    } else {
                        std::cout << "ICP did not converge." << std::endl;
                        *combined_cloud = *combined_cloud + *robots[it].cloud;
                    }
                }
            }
            else
            {
                if (use_icp_ && !leader_available)
                {
                    ROS_WARN("ICP enabled but leader cloud is unavailable. Falling back to direct merge.");
                }
                else
                {
                    ROS_INFO("ICP will not be used. Just merge all point clouds.");
                }

                for(int it = 0; it < uav_names_.size(); it++)
                {
                    if (!robots[it].cloud_received)
                    {
                        continue;
                    }
                    *combined_cloud = *combined_cloud + *robots[it].cloud;
                }
            }

            // Voxel grid filtering to merge the clouds
            PointCloudT::Ptr merged_cloud(new PointCloudT);
            pcl::VoxelGrid<PointT> voxel_filter;
            voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // Set the leaf size for downsampling
            voxel_filter.setInputCloud(combined_cloud);
            voxel_filter.filter(*merged_cloud);
            // **************** ICP Module n UAVs ****************

            // Convert the combined PCL PointCloud back to sensor_msgs/PointCloud2
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*merged_cloud, output);
            output.header.frame_id = frame_output_;  // Assuming both clouds have the same frame

            // Return the combined point cloud in the response
            res.cloud = output;
            ROS_INFO("Point clouds successfully merged in global frame.");

            // Publish global map
            map_pub.publish(output);

            // Save map
            if (save_pcd_file_ == true)
            {
                pcl::io::savePCDFileBinary(saveMapDirectory_ + "/GlobalMap.pcd", *combined_cloud);
                for(int it = 0; it < uav_names_.size(); it++)
                {
                    if (robots[it].cloud_received)
                    {
                        std::string uav_filename = robots[it].name_;
                        if (!uav_filename.empty() && uav_filename.front() == '/')
                        {
                            uav_filename.erase(0, 1);
                        }
                        save_it[it] = pcl::io::savePCDFileBinary(saveMapDirectory_ + "/Map_" + uav_filename + ".pcd", *robots[it].cloud);
                    }
                }
            }

            return true;
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher map_pub;
        ros::ServiceServer srvSaveMap;

        std::string uav_name_, name_;
        std::vector<std::string> uav_names_;

        std::string frame_output_; 
        double wait_time_;

        std::string saveMapDirectory_;
        std::string global_map_service_;

        struct singleRobot {
            int id_; // robot id
            std::string name_; // robot name
            bool cloud_received = false;
            // Use pcl::PointCloud with pcl::PointXYZ type
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            // Constructor to initialize the point cloud
            singleRobot() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {}
            ros::Subscriber map_sub_;
            // Service Client
            std::string service_name_;
            ros::ServiceClient srv_client_map;
        };

		std::vector<singleRobot> robots;
        int robot_target;

        bool save_pcd_file_;
        bool use_icp_;
        std::vector<int> save_it;

};

int main(int argc, char** argv) {
    // Initialize ROS node with automatic node name based on namespace
    ros::init(argc, argv, "map_generation_node");

    // Instantiate MapGeneration class
    MapGeneration map_generation;

    // Spin
    ros::spin();

    return 0;
}
