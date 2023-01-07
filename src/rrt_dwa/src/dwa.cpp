#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "KDTree.hpp"
#include "dwa.h"

using namespace std;

using point_t = vector< double >;
using indexArr = vector< size_t >;
using pointIndex = pair< vector< double >, size_t >;
using pointIndexArr = vector< pointIndex >;
using pointVec = vector< point_t >;


void DWA::load_param(){
    ros::param::get("w_max", w_max);
    ros::param::get("wa", wa);
    ros::param::get("w_reso", w_reso);
    ros::param::get("safe_dist", safe_dist);
    ros::param::get("penalty", penalty);
    ros::param::get("dt", dt);
    ros::param::get("predict_time", predict_time);
    ros::param::get("delay_predict_time", delay_predict_time);
    ros::param::get("goal_cost_gain", goal_cost_gain);
    ros::param::get("dist_cost_gain", dist_cost_gain);
    ros::param::get("global_path_cost_gain", global_path_cost_gain);
}


// 设定线速度
void DWA::set_v(double expected_v){
    v = expected_v;
}


// 速度窗口
vector<double> DWA::dynamic_window(double w){
    vector<double> dw_actual;
    double w_max_actual = min((w + wa * dt), w_max);
    double w_min_actual = max((w - wa * dt), -w_max);
    dw_actual = {w_min_actual, w_max_actual};
    return dw_actual;
}


// 模拟一条轨迹中位置、速度的计算
vector<double> DWA::motion_model(vector<double> x, double w, double delta_t){
    double new_orientation = x[2] + w * delta_t;
    if (new_orientation > M_PI)
        new_orientation -= 2 * M_PI;
    else if (new_orientation < -M_PI)
        new_orientation += 2 * M_PI;
    // if w == 0:  /* ‘==0’会把极小的w判到else条件里，出现问题 */
    if (w > -0.01 && w < 0.01){
        x[0] += v * delta_t * cos(x[2]);
        x[1] += v * delta_t * sin(x[2]);
    }
    else{
        double r = v / w;
        x[0] = x[0] - r * sin(x[2]) + r * sin(new_orientation);
        x[1] = x[1] + r * cos(x[2]) - r * cos(new_orientation);
    }
    x[2] = new_orientation;
    return x;
}


// 进行一条轨迹的预测
vector<vector<double>> DWA::predict_trajectory(vector<double> x, double w, double predict_time){
    vector<vector<double>> traj = {x};
    double count = 0;
    while (count <= predict_time){
        x = motion_model(x, w, dt);
        traj.push_back(x);
        count += dt;
    }
    return traj;
}


// 目标代价
double DWA::goal_cost(vector<vector<double>> &traj, point_t &goal){
    double cost = hypot(traj[traj.size()-1][0]-goal[0], traj[traj.size()-1][1]-goal[1]);
    return cost;
}


// 障碍物距离代价
double DWA::distance_cost(vector<vector<double>> &traj, pointVec &obstacle){
    double epsilon = 0.01, factor;
    // factor = 1.0;
    double min_distance = float('Inf');  // 初始化为无穷大

    KDTree obstree(obstacle);
    for (int i=0; i<traj.size(); i++){
        pointIndex res = obstree.nearest_pointIndex({traj[i][0], traj[i][1]});
        double dist = hypot(traj[i][0]-res.first[0], traj[i][1]-res.first[1]);
        if (dist < min_distance)
            min_distance = dist;
    }
    
    if (min_distance <= safe_dist)
        factor = penalty;  // 惩罚项
    else if (min_distance <= safe_dist + 0.1)
        factor = 1;
    else
        factor = 0;

    return factor / (min_distance + epsilon);
}


// 轨迹选择
pair<double, vector<vector<double>>>  DWA::control_trajectory(vector<double> &x,
        double &w, point_t &goal, pointVec &obstacle, int delay){
    
    vector<double>  dw = dynamic_window(w);
    vector<vector<double>>  best_traj = {x}, traj;
    double  best_w = w, my_w_reso;
    double  best_cost = float('inf'), cost, g_cost, dist_cost=0;
    double  i = (dw[1]-dw[0]) / w_reso;
    if (i < 5)
        my_w_reso = (dw[1]-dw[0]) / 5;
    else
        my_w_reso = w_reso;

    for (double temp_w = dw[0]; temp_w < dw[1]; temp_w += my_w_reso){
        // delay: 轨迹预测需要延长的时间
        // predict_time = predict_time + delay;
        traj = predict_trajectory(x, temp_w, predict_time);

        g_cost = goal_cost(traj, goal);
        // vel_cost = velocity_cost(v);
        if (obstacle.size())
            dist_cost = distance_cost(traj, obstacle);
        // cost = g_cost * goal_cost_gain + dist_cost * dist_cost_gain;
        cost = g_cost * goal_cost_gain;
        
        if (cost < best_cost){
            best_cost = cost;
            best_traj = traj;
            best_w = temp_w;
        }
    }

    pair<double, vector<vector<double>>> wAndTraj(best_w, best_traj);
    
    // if (best_w == 0)  // 主循环中，w==0 代表机器人第一次循环，故此处加以区别
    //     best_w = 0.0001;

    return wAndTraj;
}
