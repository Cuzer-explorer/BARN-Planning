#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "KDTree.hpp"
#include "rrt.h"
#include "dwa.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

using point_t = std::vector< double >;
using indexArr = std::vector< size_t >;
using pointIndex = typename std::pair< std::vector< double >, size_t >;
using pointIndexArr = typename std::vector< pointIndex >;
using pointVec = std::vector< point_t >;

tf::Transform map_to_base;
ros::ServiceClient map_client, state_client;
// ros::Subscriber vel_sub;
ros::Subscriber map_sub, laser_sub; // obst added
ros::Publisher vel_pub, path_pub, traj_pub, midpose_pub;

bool got_tf, got_map_obs;

// -------------- self pose,vel and destination --------------
// int RobotId;
tf::StampedTransform base_in_odom;
point_t self_pose = {-2, 3, 1.57};
point_t dst_pose = {-2, 13, 0};
double expected_v = 1.5, dwa_predict_time;
double self_w = 0;
point_t mid_goal;

// -------------- plan --------------
B_RRT global_planner;
DWA   local_planner;
pointVec  map_obs, laser_obs;
pointVec  path, temp_path, traj;
pair<double, vector<vector<double>>>  wAndTraj;
double rrt_avoid_dist;


void CB_publishCycle(const ros::TimerEvent& e);
// void CB_update_vel(const geometry_msgs::TwistStamped::ConstPtr& vel_msg);
void CB_map(const nav_msgs::OccupancyGrid& grid_msg);
void CB_laser(const sensor_msgs::LaserScan::ConstPtr& scan);
void display_PathandTraj(pointVec path, ros::Publisher visual_pub);
void display_midGoal(point_t mid_goal, ros::Publisher visual_pub);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "RRT_DWA_plan");
    ros::NodeHandle n;
    string map_topic, laser_topic, vel_pub_topic;
    // ros::param::get("RobotId", RobotId);
    ros::param::get("expected_v", expected_v);
    ros::param::get("predict_time", dwa_predict_time);  // dwa predict time
    ros::param::get("avoid_dist", rrt_avoid_dist);  // rrt avoid dist
    ros::param::get("map_topic", map_topic);
    ros::param::get("laser_topic", laser_topic);
    ros::param::get("vel_pub_topic", vel_pub_topic);
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.2), CB_publishCycle);
    global_planner.load_param();
    local_planner.load_param();

    map_client = n.serviceClient<nav_msgs::GetMap>("/static_map");
    state_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    // vel_sub = n.subscribe("/cmd_vel", 1, CB_update_vel);
    map_sub = n.subscribe(map_topic, 10, CB_map);
    laser_sub = n.subscribe(laser_topic, 10, CB_laser);

    // vel_pub = n.advertise<geometry_msgs::Twist>("target_vel", 10);
    vel_pub = n.advertise<geometry_msgs::Twist>(vel_pub_topic, 10);
    path_pub = n.advertise<nav_msgs::Path>("/course_agv/global_path", 10);
    traj_pub = n.advertise<nav_msgs::Path>("/course_agv/global_trace", 10);
    midpose_pub = n.advertise<geometry_msgs::PoseStamped>("/course_agv/mid_goal", 10);

    // ofstream ofile("/home/cuzer/BARN-Planning-master/src/rrt_dwa/log.txt");
    // if (!ofile.is_open())
    // {
    //     cerr << "Error: open file!" << endl;
    //     return -1;
    // }

    // ------------------------ debug ------------------------ //
    // self_pose = {0,0,0};
    // dst_pose = {5,5,0};
    // global_planner.set_boundary(0, 0, 10, 10);
    
    while (!got_map_obs){  // 等待读取全局地图
        cout << "reading map..." << endl;
        ros::service::waitForService("/static_map");

        nav_msgs::GetMap map_msg;
        map_client.call(map_msg);
        (cout << "map_data: " << map_msg.response.map.data[0] << ", "
         << map_msg.response.map.data[1] << ", "
         << map_msg.response.map.data[2] << ", "
         << map_msg.response.map.data[3] << ", "
         << map_msg.response.map.data[4] << ", " << endl);
        CB_map(map_msg.response.map);

        ros::Duration(0.1).sleep();
    }

    ///   RRT 规划全局路径   ///
    while (path.size() == 0){  // 如果找不到路径，减小避碰距离继续搜索
        rrt_avoid_dist -= 0.05;
        global_planner.set_avoid_dist(rrt_avoid_dist);
        double len, shortest_len = 100;
        for (int j=0; j<3; j++){
            temp_path = global_planner.plan(map_obs, {self_pose[0], self_pose[1]}, {dst_pose[0], dst_pose[1]});
            if (temp_path.size() == 0){
                continue;
            }
            len = 0;
            for (int k=0; k<temp_path.size()-1; k++)
                len += hypot(temp_path[k+1][0]-temp_path[k][0], temp_path[k+1][1]-temp_path[k][1]);
            if (len < shortest_len){
                shortest_len = len;
                path = temp_path;
            }
        }
    }
    local_planner.set_v(expected_v);  // DWA 线速度设为定值
    int goal_index = 0;  // 用于计算局部目标点
    double threshold = 1.5 * expected_v * dwa_predict_time; // 局部目标点距离
    // ------------------------------------------------------- //

    // time.sleep(500);
    



    int count=0;
    // ros::Rate r(20);
    while(ros::ok()){
        ros::spinOnce();
        
        // cout << "obst_size: " << laser_obs.size() << endl;
        /// 获取定位 ///
        // cout << "time1: " << ros::Time::now() << endl;
        ros::service::waitForService("/gazebo/get_model_state");
        gazebo_msgs::GetModelState model_state;
        model_state.request.model_name = "jackal";
        model_state.request.relative_entity_name = "world";
        state_client.call(model_state);
        if (model_state.response.success == 0){
            cout << "call model_state failed!" << endl;
            continue;
        }
        // cout << "time2: " << ros::Time::now() << endl;

        self_pose[0] = model_state.response.pose.position.x;
        self_pose[1] = model_state.response.pose.position.y;
        self_pose[2] = tf::getYaw(model_state.response.pose.orientation);
        self_w = model_state.response.twist.angular.z;

        // tf赋值
        // tf::Quaternion q;
        // tf::quaternionMsgToTF(model_state.response.pose.orientation, q);
        // map_to_base.setOrigin(tf::Vector3(self_pose[0], self_pose[1], self_pose[2]));
        // map_to_base.setRotation(q);
        // got_tf = true;
        
        if (path.size() == 0){
            continue;
        }


        ///  DWA 选取最优角速度  ///
        // 动态选取局部目标点
        int idx = goal_index;
        goal_index = path.size() - 1;
        // ——————————————————————————————————————————————
        while (idx < path.size()){
            double dis = hypot(path[idx][0]-self_pose[0], path[idx][1]-self_pose[1]);
            if (dis >= threshold){
                goal_index = idx;
                break;
            }
            idx++;
        }
        mid_goal = path[goal_index];

        // if (goal_dis >= threshold)
        double step_forward = 0.1;
        double front_point_x = path[goal_index][0];
        double front_point_y = path[goal_index][1];
        double back_point_x = path[goal_index-1][0];
        double back_point_y = path[goal_index-1][1];
        // print("front_x: %.2f, front_y: %.2f" % (front_point_x, front_point_y))
        // print("back_x: %.2f, back_y: %.2f" % (back_point_x, back_point_y))
        double front_to_back_dist = hypot(front_point_x - back_point_x,
                                        front_point_y - back_point_y);
        double ratio, temp_x, temp_y;
        
        int temp_count = int(front_to_back_dist / step_forward);
        for (int i=0; i<temp_count; i++){  // 从远至近找局部目标点
            ratio = (i+1) * step_forward / front_to_back_dist;
            temp_x = front_point_x + (back_point_x-front_point_x) * ratio;
            temp_y = front_point_y + (back_point_y-front_point_y) * ratio;
            if (hypot(temp_x-self_pose[0], temp_y-self_pose[1]) <= threshold){
                break;
            }
        }
        mid_goal[0] = temp_x;
        mid_goal[1] = temp_y;
        // ——————————————————————————————————————————————
        

        // DWA规划
        // wAndTraj = local_planner.control_trajectory(self_pose, self_w, dst_pose, laser_obs, path, 0);
        wAndTraj = local_planner.control_trajectory(self_pose, self_w, mid_goal, laser_obs, 0);
        // self_w = wAndTraj.first;
        traj = wAndTraj.second;

        // cout << "output_w: " << wAndTraj.first << ", feedback_w: " << self_w << endl;
        
        // 速度与局部目标点放在每次循环后发布
        if (mid_goal.size())
            display_midGoal(mid_goal, midpose_pub);
        
        geometry_msgs::Twist target_vel;
        if (abs(wAndTraj.first) > 0.5)  // 转弯时降低线速度
            target_vel.linear.x = 0.4;
        else if (abs(wAndTraj.first) > 0.2)
            target_vel.linear.x = 0.7;
        else
            target_vel.linear.x = expected_v;
        target_vel.angular.z = wAndTraj.first;
        vel_pub.publish(target_vel);

        // 记录相关变量信息到文件log.txt
        // ofile << "loop " << to_string(count) << endl
        //       << "goal_index: " << to_string(goal_index)
        //       << ", ratio: " << to_string(ratio)
        //       << ", mid_goal_x: " << to_string(mid_goal[0])
        //       << ", mid_goal_y: " << to_string(mid_goal[1]) << endl;

        // count++;
    }
    // ofile.close();
}


void CB_publishCycle(const ros::TimerEvent& e)
{
    // static tf::TransformBroadcaster br;
    // if (got_tf){
    //     br.sendTransform(tf::StampedTransform(map_to_base, ros::Time::now(), "/map", "/base_link"));
    // }
    if (path.size())
        display_PathandTraj(path, path_pub);
    // 局部目标点放在主函数发布
    // if (mid_goal.size())
    //     display_midGoal(mid_goal, midpose_pub);
    
    if (traj.size()){
        display_PathandTraj(traj, traj_pub);

        // 速度放在主函数发布
        // geometry_msgs::Twist target_vel;
        // if (abs(wAndTraj.first) > 0.5)  // 转弯时降低线速度
        //     target_vel.linear.x = 0.5;
        // else if (abs(wAndTraj.first) > 0.2)
        //     target_vel.linear.x = 0.8;
        // else
        //     target_vel.linear.x = expected_v;
        // target_vel.angular.z = wAndTraj.first;
        // vel_pub.publish(target_vel);
    }
}


// void  CB_update_vel(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)  // 速度反馈
// {
//     self_w = vel_msg->twist.angular.z;
// }


// void CB_map(const nav_msgs::OccupancyGrid::ConstPtr& grid_msg) // 全局地图
void CB_map(const nav_msgs::OccupancyGrid& grid_msg) // 全局地图
{
    if (got_map_obs)
        return;

    double map_origin_x = grid_msg.info.origin.position.x;
    double map_origin_y = grid_msg.info.origin.position.y;
    double resolution = grid_msg.info.resolution;
    int height = grid_msg.info.height;
    int width = grid_msg.info.width;

    for (int i=0; i<height; i++){
        for (int j=0; j<width; j++){
            // 上下左右都占据的栅格，忽略
            if(i>0 && i<height-1 && j>0 && j<width-1){
                if(grid_msg.data[i*width + j-1] == 100 &&
                   grid_msg.data[i*width + j+1] == 100 &&
                   grid_msg.data[(i-1)*width + j] == 100 &&
                   grid_msg.data[(i+1)*width + j] == 100)
                {
                    continue;
                }
            }
            if(grid_msg.data[i*width + j] == 100){
                map_obs.push_back(
                    {j*resolution + resolution/2 + map_origin_x,
                     i*resolution + resolution/2 + map_origin_y}
                );
            }
        }
    }

    got_map_obs = 1;
}

void CB_laser(const sensor_msgs::LaserScan::ConstPtr& scan_msg)  // 激光
{
    static tf::TransformListener tf_listener;
    double obst_dist = 5.0;

    laser_obs.clear();

    int scan_size = scan_msg->ranges.size();
    // cout << "angle_min: " << scan_msg->angle_min << endl;
    // cout << "angle_max: " << scan_msg->angle_max << endl;
    // cout << "angle_increment: " << scan_msg->angle_increment << endl;
    // cout << "range_min: " << scan_msg->range_min << endl;
    // cout << "range_max: " << scan_msg->range_max << endl;

    int last_i;
    for(int i = 0; i < scan_size; i++)  // for one point
    // for(int i = 0; i < scan_size; i+=5)  // for one point
    {
        double range = scan_msg->ranges[i];

        // cout << "range " << i << ": " << range << ", ";

        if( isinf(range) || isnan(range) )
        {
            continue;
        }
        else if( range < obst_dist )  // in the inflation range
        {
            // only consider obst in front of car
            if(scan_msg->angle_min + i*scan_msg->angle_increment < -M_PI/2  ||  scan_msg->angle_min + i*scan_msg->angle_increment > M_PI/2)
                continue;
            
            double angle = scan_msg->angle_min + i * (scan_msg->angle_increment);
            double x = range * cos(angle);
            double y = range * sin(angle);

            geometry_msgs::PointStamped obst_in_laser;
            geometry_msgs::PointStamped obst_in_world;

            obst_in_laser.header.stamp = ros::Time();
            // obst_in_laser.header.frame_id = scan_msg->header.frame_id;  // laser
            obst_in_laser.header.frame_id = "/base_link";  // laser
            obst_in_laser.point.x = x;
            obst_in_laser.point.y = y;
            obst_in_laser.point.z = 0;
            try{
                tf_listener.transformPoint("/map", obst_in_laser, obst_in_world);
            }
            catch (tf::TransformException &ex){
                ROS_ERROR("%s, in CB_laser.", ex.what());
                return;
            }

            laser_obs.push_back({obst_in_world.point.x, obst_in_world.point.y});
        }
        else
        {
            continue;
        }
    }

    // got_laser_obst = 1;
}


void display_PathandTraj(pointVec path, ros::Publisher visual_pub){  // 在 Rviz 中显示 RRT 与 DWA 路径
    nav_msgs::Path  visual_path;
    visual_path.header.stamp = ros::Time(0);
    visual_path.header.frame_id = "/map";
    for (int i = 0; i < path.size(); i++){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time(0);
        pose.header.frame_id = "/map";
        pose.pose.position.x = path[i][0];
        pose.pose.position.y = path[i][1];
        pose.pose.position.z = 0.01;
        pose.pose.orientation.w = 1;
        visual_path.poses.push_back(pose);
    }
    visual_pub.publish(visual_path);
}

void display_midGoal(point_t mid_goal, ros::Publisher visual_pub){  // Rviz 中显示局部目标点
    geometry_msgs::PoseStamped  mid_goal_msg;
    mid_goal_msg.header.stamp = ros::Time(0);
    mid_goal_msg.header.frame_id = "/map";
    mid_goal_msg.pose.position.x = mid_goal[0];
    mid_goal_msg.pose.position.y = mid_goal[1];
    mid_goal_msg.pose.orientation.w = 1;

    visual_pub.publish(mid_goal_msg);
}
