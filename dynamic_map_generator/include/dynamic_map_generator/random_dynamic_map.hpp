/**
 * @file random_dynamic_map.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __RANDOM_DYNAMIC_MAP_H__
#define __RANDOM_DYNAMIC_MAP_H__

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <dynamic_map_generator/obstacles.hpp>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <random>
#include <string>
#include <vector>

class RandomDynamicMap
{
private:
    ros::NodeHandle node_;
    ros::Publisher all_map_cloud_pub_;
    ros::Publisher obstacle_vis_pub_;
    ros::Publisher obstacle_state_pub_;
    ros::Publisher local_map_cloud_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer vis_timer_;

    /* map parameters */
    float x_size_, y_size_, z_size_;          // map size
    float x_l_, x_h_, y_l_, y_h_, z_l_, z_h_; // map boundary
    float v_h_;                               // vel max
    float resolution_;
    float sensing_rate_;
    float sensing_range_;
    std::string frame_id_;

    /* random seed */
    int seed_;
    std::default_random_engine eng_;

    /* obstacles */
    int num_obstacles_;
    std::vector<std::unique_ptr<Obstacle>> obstacles_;

    AABBConfig aabb_config_;
    CylinderConfig cylinder_config_;
    CircleGateConfig circlegate_config_;

    /* whole point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    bool is_rendered_{false};
    bool enable_ground_{false};


    /* odom  && local map*/
    std::string odom_topic_;




public:
    RandomDynamicMap(ros::NodeHandle &nh): node_(nh){}
    ~RandomDynamicMap(){}

    void init();
    void renderMap();
    void publishMap();
    void getGlobalMap();
    void pubslishLocalMap();
    void publishObstacleState();
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void visCallback(const ros::TimerEvent &event);

    Eigen::Vector3f sampleRandomPosition();
    Eigen::Vector3f sampleRandomVelocity2D();
    Eigen::Vector3f sampleRandomVelocity();

  /* helper functions */
    inline float getSenseRate() const { return sensing_rate_; }
    inline float getSenseRange() const {return sensing_range_; }
    inline float uniformSample(float max, float min) {
        return std::uniform_real_distribution<float>(min, max)(eng_);
    }

};

#endif // __RANDOM_DYNAMIC_MAP_H__