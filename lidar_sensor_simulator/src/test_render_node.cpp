#include "opengl_sim.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <deque>
#include <numeric>
#include <ros/package.h>
#include <string>
#include <tr1/unordered_map>

using namespace Eigen;
using namespace std;

#define MAX_INTENSITY 1
#define MIN_INTENSITY 0.1

string file_name, pkg_path;
std::ofstream myfile, collision_checktime_file;
deque<double> comp_time_vec;

std::string quad_name;
int drone_num = 0;
int drone_id = 0;

opengl_pointcloud_render render;


ros::Publisher pub_cloud, pub_pose, pub_intercloud, pub_dyncloud, pub_uavcloud, depth_img_pub_, comp_time_pub; /*pub_collisioncloud*/
sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;
ros::Subscriber odom_sub;
ros::Subscriber static_map_sub, dynamic_map_sub;
ros::Timer local_sensing_timer, pose_timer, dynobj_timer;

ros::Time t_init;

bool has_odom_(false);
bool has_dynamic_map_(false);
bool has_static_map_(false);

nav_msgs::Odometry odom_;
Eigen::Matrix4f sensor2body, sensor2world;
// pcl::PointCloud<pcl::PointType> static_map, dynamic_map;
pcl::PointCloud<PointType> dynamic_map;
pcl::PointCloud<PointType> static_map;

pcl::PointCloud<PointType>::Ptr local_map(new pcl::PointCloud<PointType>); // 存储局部点云
sensor_msgs::PointCloud2 local_map_pcd, sensor_map_pcd; // 输出局部感知点云,1.世界坐标系下, 2.传感器坐标系下

/* parameters */
int output_pcd; // 
// int collisioncheck_enable; // 是否对无人机进行障碍物检测
int is_360lidar; // 是否使用mid360
int use_avia_pattern, use_vlp32_pattern, use_minicf_pattern, use_os128_pattern, use_gaussian_filter;
int livox_linestep;
double sensing_horizon, sensing_rate, estimation_rate, polar_resolution, yaw_fov, vertical_fov, min_raylength, downsample_res, curvature_limit, hash_cubesize, collision_range;
double x_size, y_size, z_size;
double gl_xl, gl_yl, gl_zl;
double resolution, inv_resolution;
int GLX_SIZE, GLY_SIZE, GLZ_SIZE;

pcl::PointCloud<PointType> cloud_all_map, local_map_filled, point_in_sensor;




void rcvDynamicMapCallBack(const sensor_msgs::PointCloud2 &dynamic_map_in)
{
    ROS_WARN("[Lidar Simulator] Dynamic Pointcloud received.. ");
    dynamic_map.clear();
    pcl::fromROSMsg(dynamic_map_in, dynamic_map);
    
}


void rcvStaticMapCallBack(const sensor_msgs::PointCloud2 &static_map_in)
{
    // 只接收一次静态点云
    if(has_static_map_)
    {
        return; 
    }

    ROS_WARN("[Lidar Simulator] Static Pointcloud received..");

    pcl::fromROSMsg(static_map_in, static_map);
    render.read_pointcloud(static_map);

    has_static_map_ = true;
}


void rcvOdometryCallbck(const nav_msgs::Odometry &odom)
{

    ROS_WARN("[Lidar Simulator] odom received..");

    has_odom_ = true;
    odom_ = odom;

    Matrix4f body2world = Matrix4f::Identity();

    Eigen::Vector3f request_position;
    Eigen::Quaternionf pose;
    pose.x() = odom.pose.pose.orientation.x;
    pose.y() = odom.pose.pose.orientation.y;
    pose.z() = odom.pose.pose.orientation.z;
    pose.w() = odom.pose.pose.orientation.w;
    body2world.block<3, 3>(0, 0) = pose.toRotationMatrix();
    body2world(0, 3) = odom.pose.pose.position.x;
    body2world(1, 3) = odom.pose.pose.position.y;
    body2world(2, 3) = odom.pose.pose.position.z;

    // convert to cam pose
    sensor2world = body2world * sensor2body;

}

int comp_time_count = 0;
void renderSensedPoints(const ros::TimerEvent &event)
{
    ros::Time t1 = ros::Time::now();
    if (!has_odom_)
    {
        return;
    }

    Eigen::Quaternionf q;
    q.x() = odom_.pose.pose.orientation.x;
    q.y() = odom_.pose.pose.orientation.y;
    q.z() = odom_.pose.pose.orientation.z;
    q.w() = odom_.pose.pose.orientation.w;
    const Eigen::Matrix3f rot(q.toRotationMatrix());
    const Eigen::Vector3f pos(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);

    ros::Time time_stamp_ = odom_.header.stamp;

    ros::Time t_pattern = ros::Time::now();
    double time_frominit = (t_pattern - t_init).toSec();

    local_map->points.clear();

    if(dynamic_map.points.size() > 0)
    {
        // render 输入接收到的动态点云
        render.input_dyn_clouds(dynamic_map);
        sensor_msgs::PointCloud2 dynamic_points_pcd;
        pcl::toROSMsg(dynamic_map, dynamic_points_pcd);
        dynamic_points_pcd.header.frame_id = "world";
        dynamic_points_pcd.header.stamp = ros::Time::now();
        pub_dyncloud.publish(dynamic_points_pcd);
    }
    // trans and publish the dynamic point cloud, 发布动态点云

    render.render_pointcloud(local_map, pos, q, time_frominit);

    ros::Time t_afterrender = ros::Time::now();
    double comp_time_temp = (t_afterrender - t1).toSec();
    myfile << comp_time_temp << endl;
    comp_time_vec.push_back(comp_time_temp);
    if (comp_time_count > 20)
    {
        comp_time_vec.pop_front();
        geometry_msgs::PoseStamped totaltime_pub;
        totaltime_pub.pose.position.x = accumulate(comp_time_vec.begin(), comp_time_vec.end(), 0.0) / comp_time_vec.size();
        comp_time_pub.publish(totaltime_pub);
        ROS_INFO("Temp compute time = %lf, average compute time = %lf", comp_time_temp, totaltime_pub.pose.position.x);
    }
    else
    {
        comp_time_count++;
    }

    local_map->width = local_map->points.size();
    local_map->height = 1;
    local_map->is_dense = true;

    /* 发布感知点云 */
    pcl::toROSMsg(*local_map, local_map_pcd);
    local_map_pcd.header = odom_.header;
    local_map_pcd.header.stamp = time_stamp_;
    local_map_pcd.header.frame_id = "world";
    pub_cloud.publish(local_map_pcd);

    Eigen::Matrix4f sensor2world;
    sensor2world << rot(0, 0), rot(0, 1), rot(0, 2), pos.x(),
        rot(1, 0), rot(1, 1), rot(1, 2), pos.y(),
        rot(2, 0), rot(2, 1), rot(2, 2), pos.z(),
        0, 0, 0, 1;
    Eigen::Matrix4f world2sensor;
    world2sensor = sensor2world.inverse();

    // body frame pointcloud
    point_in_sensor.points.clear();
    // pcl::copyPointCloud(local_map_filled, point_in_sensor);
    pcl::transformPointCloud(*local_map, point_in_sensor, world2sensor);
    point_in_sensor.width = point_in_sensor.points.size();
    point_in_sensor.height = 1;
    point_in_sensor.is_dense = true;

    std::string sensor_frame_id_ = "/sensor";
    pcl::toROSMsg(point_in_sensor, sensor_map_pcd);
    sensor_map_pcd.header.frame_id = sensor_frame_id_;
    sensor_map_pcd.header.stamp = time_stamp_;
    pub_intercloud.publish(sensor_map_pcd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_render");
    ros::NodeHandle nh("~");

    /* params */
    nh.getParam("is_360lidar", is_360lidar);
    nh.getParam("sensing_horizon", sensing_horizon);
    nh.getParam("sensing_rate", sensing_rate);
    nh.getParam("estimation_rate", estimation_rate);
    nh.getParam("polar_resolution", polar_resolution);
    nh.getParam("yaw_fov", yaw_fov);
    nh.getParam("vertical_fov", vertical_fov);
    nh.getParam("min_raylength", min_raylength);
    nh.getParam("downsample_res", downsample_res);
    nh.getParam("livox_linestep", livox_linestep);
    nh.getParam("use_avia_pattern", use_avia_pattern);
    nh.getParam("curvature_limit", curvature_limit);
    nh.getParam("hash_cubesize", hash_cubesize);
    nh.getParam("use_vlp32_pattern", use_vlp32_pattern);
    nh.getParam("use_minicf_pattern", use_minicf_pattern);
    nh.getParam("use_os128_pattern", use_os128_pattern);
    nh.getParam("use_gaussian_filter", use_gaussian_filter);
    nh.getParam("output_pcd", output_pcd);

    nh.getParam("map/x_size", x_size);
    nh.getParam("map/y_size", y_size);
    nh.getParam("map/z_size", z_size);

    // render.setParameters(400,400,250,250,downsample_res,0.1,sensing_horizon,sensing_rate,use_avia_pattern);
    int image_width;
    int image_height;
    if (is_360lidar)
    {
        image_width = ceil(360.0 / polar_resolution);
    }
    else
    {
        image_width = ceil(yaw_fov / polar_resolution);
    }
    image_height = ceil(vertical_fov / polar_resolution);
    render.setParameters(image_width, image_height, 250, 250, downsample_res, polar_resolution, yaw_fov, vertical_fov, 0.1, sensing_horizon, sensing_rate, use_avia_pattern, use_os128_pattern, use_minicf_pattern);

    // 读取全局点云地图
    // file_name = argv[1];
    // render.read_pointcloud_fromfile(file_name);


    odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);
    // local_cloud_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
    dynamic_map_sub = nh.subscribe("dynamic_map", 1, rcvDynamicMapCallBack);
    static_map_sub = nh.subscribe("static_map", 1, rcvStaticMapCallBack);


    pub_dyncloud = nh.advertise<sensor_msgs::PointCloud2>("dyn_cloud", 10);
    pub_intercloud = nh.advertise<sensor_msgs::PointCloud2>("sensor_cloud", 10);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 10);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("sensor_pose", 10);
    // pub_uavcloud = nh.advertise<sensor_msgs::PointCloud2>("uav_cloud", 10);
    // depth_img_pub_ = nh.advertise<sensor_msgs::Image>("depth_img", 10);
    // comp_time_pub = nh.advertise<geometry_msgs::PoseStamped>("simulator_compute_time", 10);
    // pub_collisioncloud = nh.advertise<sensor_msgs::PointCloud2>("collision_cloud", 10);
    double sensing_duration = 1.0 / sensing_rate;
    double estimate_duration = 1.0 / estimation_rate;

    /* 打开记录数据句柄 */
    pkg_path = ros::package::getPath("lidar_sensing_node");
    pkg_path.append("/data/" + quad_name + "_GPU_time_consumption.txt"); // append 追加
    std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
    myfile.open(pkg_path.c_str(), std::ios_base::out);

    // open file to record collision check time consumption
    pkg_path = ros::package::getPath("lidar_sensing_node");
    pkg_path.append("/data/" + quad_name + "_GPU_collision_check_time_consumption.txt");
    std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
    collision_checktime_file.open(pkg_path.c_str(), std::ios_base::out);

    ros::Rate rate(100);
    bool status = ros::ok();

    while (status)
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}