#include <dynamic_map_generator/random_dynamic_map.hpp>

void RandomDynamicMap::init()
{
    node_.param("map/seed", seed_, 0);

    node_.param("map/x_size", x_size_, 10.0F);
    node_.param("map/y_size", y_size_, 10.0F);
    node_.param("map/z_size", z_size_, 5.0F);
    node_.param("map/obs_num", num_obstacles_, 30);
    node_.param("map/resolution", resolution_, 0.1F);
    node_.param("map/frame_id", frame_id_, std::string("world"));
    node_.param("sensing/rate", sensing_rate_, 10.0F);
    node_.param("sensing/range", sensing_range_, 5.0F);


    node_.param("obstacle/upper_vel", v_h_, 0.1F);
    node_.param("aabb/lower_x", aabb_config_.size_x_min, 0.1F);
    node_.param("aabb/upper_x", aabb_config_.size_x_max, 0.5F);
    node_.param("aabb/lower_y", aabb_config_.size_y_min, 0.1F);
    node_.param("aabb/upper_y", aabb_config_.size_y_max, 0.5F);
    node_.param("aabb/lower_z", aabb_config_.size_z_min, 0.1F);
    node_.param("aabb/upper_z", aabb_config_.size_z_max, 0.5F);
    node_.param("cylinder/lower_rad", cylinder_config_.radius_min, 0.3F);
    node_.param("cylinder/upper_rad", cylinder_config_.radius_max, 0.8F);
    node_.param("cylinder/lower_hei", cylinder_config_.height_min, 3.0F);
    node_.param("cylinder/upper_hei", cylinder_config_.height_max, 4.0F);
    node_.param("circlegate/radius_l", circlegate_config_.radius_min, 7.0F);
    node_.param("circlegate/radius_h", circlegate_config_.radius_max, 7.0F);
    node_.param("circlegate/thickness", circlegate_config_.thickness_max, 7.0F);
    node_.param("circlegate/dr", circlegate_config_.alpha_max, 0.2F);
    node_.param("circlegate/theta", circlegate_config_.theta_max, 2.0F);

    node_.param("odom_topic", odom_topic_, std::string("odom"));
    node_.param<bool>("enable_ground", enable_ground_, false);
    // ROS_INFO("map/seed: %d", seed_);
    // ROS_INFO("map/future_num: %d", num_future_map_);
    // ROS_INFO("map/time_step: %f", future_step_size_);
    // ROS_INFO("map/x_size: %f", x_size_);
    // ROS_INFO("map/y_size: %f", y_size_);
    // ROS_INFO("map/z_size: %f", z_size_);
    // ROS_INFO("map/test: %d", is_test_mode_);
    // ROS_INFO("map/obs_num: %d", num_obstacles_);
    // ROS_INFO("map/resolution: %f", resolution_);
    // ROS_INFO_STREAM("map/frame_id: " << frame_id_);
    // ROS_INFO("sensing/rate: %f", sense_rate_);
    // ROS_INFO("mode: %d", mode_);

    // ROS_INFO("obstacle/upper_vel: %f", v_h_);
    // ROS_INFO("aabb/lower_x: %f", aabb_config_.size_x_min);
    // ROS_INFO("aabb/upper_x: %f", aabb_config_.size_x_max);
    // ROS_INFO("aabb/lower_y: %f", aabb_config_.size_y_min);
    // ROS_INFO("aabb/upper_y: %f", aabb_config_.size_y_max);
    // ROS_INFO("aabb/lower_z: %f", aabb_config_.size_z_min);
    // ROS_INFO("aabb/upper_z: %f", aabb_config_.size_z_max);
    // ROS_INFO("cylinder/lower_rad: %f", cylinder_config_.radius_min);
    // ROS_INFO("cylinder/upper_rad: %f", cylinder_config_.radius_max);
    // ROS_INFO("cylinder/lower_hei: %f", cylinder_config_.height_min);
    // ROS_INFO("cylinder/upper_hei: %f", cylinder_config_.height_max);
    // ROS_INFO("circlegate/radius_l: %f", circlegate_config_.radius_min);
    // ROS_INFO("circlegate/radius_h: %f", circlegate_config_.radius_max);
    // ROS_INFO("circlegate/thickness: %f", circlegate_config_.thickness_max);
    // ROS_INFO("circlegate/dr: %f", circlegate_config_.alpha_max);
    // ROS_INFO("circlegate/theta: %f", circlegate_config_.theta_max);

    /* initialize random seed */
    if (seed_ == 0)
    {
        std::random_device rd;
        eng_ = std::default_random_engine(rd());
    }
    else
    {
        eng_ = std::default_random_engine(seed_);
    }

    x_l_ = -x_size_ / 2;
    x_h_ = x_size_ / 2;
    y_l_ = -y_size_ / 2;
    y_h_ = y_size_ / 2;
    z_l_ = 0;
    z_h_ = z_size_;

    // ros::Duration(2.0).sleep(); /* sleep for 1s to wait for rviz */
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    all_map_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
    local_map_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("local_cloud", 1);
    obstacle_vis_pub_ = node_.advertise<visualization_msgs::MarkerArray>("global_cylinders_vis", 1);
    obstacle_state_pub_ = node_.advertise<visualization_msgs::MarkerArray>("global_cylinders_state", 1);

    vis_timer_ = node_.createTimer(ros::Duration(1.0 / sensing_rate_), &RandomDynamicMap::visCallback, this);
    odom_sub_ = node_.subscribe(odom_topic_, 1, &RandomDynamicMap::odomCallback, this);
    ROS_WARN("ODOM TOPIC: %s", odom_topic_.c_str());
    /* ---- Initialize Obstacles ---- */
    obstacles_.reserve(num_obstacles_);
    // for(int i=0;i<num_obstacles_;i++)
    // {
    //     float size_x = uniformSample(aabb_config_.size_x_max, aabb_config_.size_x_min);
    //     float size_y = uniformSample(aabb_config_.size_y_max, aabb_config_.size_y_min);
    //     float size_z = uniformSample(aabb_config_.size_z_max, aabb_config_.size_z_min);
    //     Eigen::Vector3f size(size_x, size_y, size_z);
    //     obstacles_.emplace_back(
    //         std::make_unique<AABB>(sampleRandomPosition(), sampleRandomVelocity2D(), size));
    //     std::cout << "AABB size: " << size.transpose() << std::endl;
    // }
    for (int i = 0; i < num_obstacles_; i++)
    {
        int obstacle_type = std::rand() % 2;
        switch (obstacle_type)
        {
            case 0: /* AABB */
            {
                float size_x = uniformSample(aabb_config_.size_x_max, aabb_config_.size_x_min);
                float size_y = uniformSample(aabb_config_.size_y_max, aabb_config_.size_y_min);
                float size_z = uniformSample(aabb_config_.size_z_max, aabb_config_.size_z_min);
                Eigen::Vector3f size(size_x, size_y, size_z);
                obstacles_.emplace_back(
                    std::make_unique<AABB>(sampleRandomPosition(), sampleRandomVelocity2D(), size));
                std::cout << "AABB size: " << size.transpose() << std::endl;
                break;
            }
            case 1: /* Cylinder */
            {
                float radius = uniformSample(cylinder_config_.radius_max, cylinder_config_.radius_min);
                float height = cylinder_config_.height_max;

                Eigen::Vector3f p = sampleRandomPosition();
                obstacles_.emplace_back(
                    std::make_unique<Cylinder>(p, sampleRandomVelocity2D(), radius, height));
                std::cout << "Cylinder radius: " << radius << " height: " << height << std::endl;
                break;
            }
            // case 2: /* Circle Gate */
            // {
            //     float radius = uniformSample(circlegate_config_.radius_max, circlegate_config_.radius_min);
            //     float thickness = circlegate_config_.thickness_max;
            //     float alpha = circlegate_config_.alpha_max;
            //     float theta = uniformSample(circlegate_config_.theta_max, -circlegate_config_.theta_max);
            //     obstacles_.emplace_back(std::make_unique<CircleGate>(
            //         sampleRandomPosition(), sampleRandomVelocity2D(), radius, thickness, alpha, theta));
            //     std::cout << "CircleGate radius: " << radius << " thickness: " << thickness
            //             << " alpha: " << alpha << " theta: " << theta << std::endl;
            //     break;
            // }
        }
    }

    if(enable_ground_)
    {
        Eigen::Vector3f size{x_size_,y_size_,0.01};
        obstacles_.emplace_back(std::make_unique<Ground>(size));
    }

    /* render obstacles */
}


void RandomDynamicMap::renderMap() {
    /* render obstacles */
    ROS_INFO("Rendering map...");
    ros::Time t0 = ros::Time::now();
    for (auto& obs : obstacles_) {
        ros::Time t1 = ros::Time::now();
        obs->render(resolution_);
        std::cout << "Obstacle render cost: " << ros::Time::now() - t1 << std::endl;
    }
    ROS_INFO("Map rendered, cost: %f", (ros::Time::now() - t0).toSec());
    is_rendered_ = true;
}

void RandomDynamicMap::publishMap() {
    if (!is_rendered_) {
        ROS_WARN("Map is not rendered yet!");
        return;
    }
    float dt = 1 / sensing_rate_;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_->clear();
    for (auto& obs : obstacles_) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZ>);
        obs->update(dt,x_l_,x_h_,y_l_,y_h_,z_l_,z_h_);
        obs->getCloud(cloud_obs);
        *cloud_ += *cloud_obs;
    }
    cloud_->width    = cloud_->points.size();
    cloud_->height   = 1;
    cloud_->is_dense = true;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_, cloud_msg);
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.header.stamp    = ros::Time::now();
    all_map_cloud_pub_.publish(cloud_msg);
}


/**
 * @brief publish the visualization of the obstacles
 *
 * @NOTE: please make sure the obstacles are updated before calling this function
 */
void RandomDynamicMap::publishObstacleState() {

    if (!is_rendered_) {
        ROS_WARN("Map is not rendered yet!");
        return;
    }
    visualization_msgs::MarkerArray obstacle_state_list;
    obstacle_state_list.markers.reserve(obstacles_.size());

    int id = 0;


    for (auto& obs : obstacles_) {
        Eigen::Vector3f p, v, s;
        p                    = obs->getPosition();
        v                    = obs->getVelocity();
        s                    = obs->getBBox();
        Eigen::Quaternionf q = obs->getQuaternion();

        visualization_msgs::Marker arrow;
        arrow.header.stamp = ros::Time::now();
        arrow.header.frame_id = frame_id_;
        arrow.ns = "velocity";
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start,end;
        start.x = p.x();
        start.y = p.y();
        start.z = p.z() + s.z() ;
        end.x = p.x() + v.x();
        end.y = p.y() + v.y();
        end.z = p.z() + v.z() + s.z() ;
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        arrow.pose.orientation.x = q.x();
        arrow.pose.orientation.y = q.y();
        arrow.pose.orientation.z = q.z();
        arrow.pose.orientation.w = q.w();
        

        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;

        arrow.scale.x = 0.1;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.id = id;

        obstacle_state_list.markers.push_back(arrow);   


        visualization_msgs::Marker box;
        box.header.stamp = ros::Time::now();
        box.header.frame_id = frame_id_;
        box.type = visualization_msgs::Marker::CUBE;
        box.action = visualization_msgs::Marker::ADD;

        box.pose.position.x    = p.x();
        box.pose.position.y    = p.y();
        box.pose.position.z    = p.z() + s.z() / 2;
        box.pose.orientation.x = q.x();
        box.pose.orientation.y = q.y();
        box.pose.orientation.z = q.z();
        box.pose.orientation.w = q.w();

        box.scale.x = s.x();
        box.scale.y = s.y();
        box.scale.z = s.z();

        box.color.r = 1.0;
        box.color.g = 0.0;
        box.color.b = 0.0;
        box.color.a = 0.5;
        box.id = id;

        obstacle_state_list.markers.push_back(box);

        id++;
    }
    obstacle_state_pub_.publish(obstacle_state_list);
}

void RandomDynamicMap::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{

    if (!is_rendered_) 
    {
        ROS_WARN("Map is not rendered yet!");
        return;
    }

    /*  邻域搜索 */
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 定义搜索点
    pcl::PointXYZ searchPoint;
    searchPoint.x = msg->pose.pose.position.x;
    searchPoint.y = msg->pose.pose.position.y;
    searchPoint.z = msg->pose.pose.position.z;

    // 定义搜索半径
    float radius = sensing_range_;

    // 定义两个向量来存储搜索到的点的索引和对应的距离平方
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // 执行半径搜索，搜索半径为radius的邻域内的点
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
            local_cloud->points.push_back(cloud_->points[pointIdxRadiusSearch[i]]);
        }
    }
    local_cloud->width    = local_cloud->points.size();
    local_cloud->height   = 1;
    local_cloud->is_dense = true;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*local_cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.header.stamp    = msg->header.stamp;
    local_map_cloud_pub_.publish(cloud_msg);

}

void RandomDynamicMap::visCallback(const ros::TimerEvent &event)
{
    publishMap();
    publishObstacleState();
}

Eigen::Vector3f RandomDynamicMap::sampleRandomPosition() {
    std::uniform_real_distribution<float> rand_x(x_l_, x_h_);
    std::uniform_real_distribution<float> rand_y(y_l_, y_h_);
    // std::uniform_real_distribution<float> rand_z(z_l_, z_h_);
    return Eigen::Vector3f(rand_x(eng_), rand_y(eng_),0);
}

Eigen::Vector3f RandomDynamicMap::sampleRandomVelocity2D() {
    std::uniform_real_distribution<float> rand_v(-v_h_, v_h_);
    std::uniform_real_distribution<float> rand_omega(-M_PI, M_PI);
    return Eigen::Vector3f(rand_v(eng_) * std::cos(rand_omega(eng_)),
                            rand_v(eng_) * std::sin(rand_omega(eng_)), 0.0F);
}
