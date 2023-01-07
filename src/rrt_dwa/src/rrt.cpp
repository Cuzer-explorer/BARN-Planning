#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "KDTree.hpp"
#include "rrt.h"

using namespace std;

using point_t = vector< double >;
using indexArr = vector< size_t >;
using pointIndex = pair< vector< double >, size_t >;
using pointIndexArr = vector< pointIndex >;
using pointVec = vector< point_t >;



void B_RRT::load_param(){
    ros::param::get("N_SAMPLE", N_SAMPLE);
    ros::param::get("minx", minx);
    ros::param::get("maxx", maxx);
    ros::param::get("miny", miny);
    ros::param::get("maxy", maxy);
    ros::param::get("robot_size", robot_size);
    ros::param::get("avoid_dist", avoid_dist);
    ros::param::get("steer_dist", steer_dist);
    ros::param::get("allow_path_theta", allow_path_theta);
}


pointVec B_RRT::plan(pointVec &obstacle, point_t start, point_t goal){
    if (obstacle.size() == 0){
        return {start, goal};
    }
    // 障碍物KD树
    KDTree obstree(obstacle);

    // RRT初始化
    // '1' 代表从起点开始的 RRT, '2' 代表从终点开始的 RRT (两个RRT会交换)
    pointVec  RRTnode_1, RRTnode_2, path, path_1, path_2, smoothest_path, pruning_path;
    indexArr  child_to_parent_1, child_to_parent_2;

    RRTnode_1.push_back(start);
    RRTnode_2.push_back(goal);
    child_to_parent_1.push_back(0);
    child_to_parent_2.push_back(0);

    point_t sample={0,0}, near_1, new_1, near_2, new_2, newer_2;
    pointIndex res;
    int count = 0, success = 0;
    double dist;
    size_t index;

    // srand((int)time(0));
    srand(rand());  // 产生随机种子，避免每次随机数序列都相同

    while(success == 0){ // 没有找到符合要求的路径
        // 循环次数 +1
        count++;
        
        // 交换两个RRT来平衡两棵树的规模
        if(RRTnode_1.size() < RRTnode_2.size()){
            swap(RRTnode_1, RRTnode_2);
            swap(child_to_parent_1, child_to_parent_2);
        }
        // 如果循环次数超出限制，停止搜索并返回目前为止最平滑的RRT
        if(count >= N_SAMPLE){
            if(smoothest_path.size()==0)
                ROS_INFO("Path searching failed!");
            else if(smoothest_path[0] == goal)
                reverse(smoothest_path.begin(), smoothest_path.end());
            return smoothest_path;
        }
        // 采样 RRT1
        sample[0] = (rand()/(double)RAND_MAX * (maxx - minx)) + minx;
        sample[1] = (rand()/(double)RAND_MAX * (maxy - miny)) + miny;

        res = obstree.nearest_pointIndex(sample);
        dist = hypot(sample[0]-res.first[0], sample[1]-res.first[1]);
        index = res.second;
        if(dist < robot_size + avoid_dist)
            continue;
        
        // 寻找 RRT1 中的最近点
        KDTree RRT_tree_1(RRTnode_1);
        res = RRT_tree_1.nearest_pointIndex(sample);
        dist = hypot(sample[0]-res.first[0], sample[1]-res.first[1]);
        index = res.second;
        near_1 = res.first;
        
        // 生成 RRT1 中的新节点
        new_1 = steer(near_1, sample);

        if (check_obs(near_1, new_1, obstree) == 0){
            RRTnode_1.push_back(new_1);
            child_to_parent_1.push_back(index);
            
            // 向 RRT1 新节点的方向扩展 RRT2
            KDTree RRT_tree_2(RRTnode_2);
            res = RRT_tree_2.nearest_pointIndex(new_1);
            dist = hypot(new_1[0]-res.first[0], new_1[1]-res.first[1]);
            index = res.second;
            near_2 = res.first;
            new_2 = steer(near_2, new_1);
            
            if (check_obs(near_2, new_2, obstree) == 0){
                RRTnode_2.push_back(new_2);
                child_to_parent_2.push_back(index);
                // 不断扩展 RRT2 直到发生碰撞或与 RRT1 相连
                while (hypot(new_2[0]-new_1[0], new_2[1]-new_1[1]) > steer_dist){
                    newer_2 = steer(new_2, new_1);
                    if (check_obs(new_2, newer_2, obstree) == 0){
                        RRTnode_2.push_back(newer_2);
                        child_to_parent_2.push_back(RRTnode_2.size()-2);
                        new_2 = newer_2;
                    }
                    else
                        break;
                }
            }
            else
                continue;
                    
            if (hypot(new_2[0]-new_1[0], new_2[1]-new_1[1]) <= steer_dist){
                // 成功条件1
                if (check_obs(new_2, new_1, obstree) == 0){
                    path_1 = find_path(RRTnode_1, child_to_parent_1);
                    path_2 = find_path(RRTnode_2, child_to_parent_2);
                    reverse(path_1.begin(), path_1.end());
                    // 合并两条路径
                    path.insert(path.end(), path_1.begin(), path_1.end());
                    path.insert(path.end(), path_2.begin(), path_2.end());
                    pruning_path = pruning(path, obstree);
                    
                    // 保存最平滑的路径
                    if (min_path_theta(pruning_path) > min_path_theta(smoothest_path)){
                        smoothest_path.clear(); smoothest_path.shrink_to_fit();
                        smoothest_path.insert(smoothest_path.end(), pruning_path.begin(), pruning_path.end());
                    }
                    
                    // 成功条件2
                    if (min_path_theta(pruning_path) < allow_path_theta){
                        // 如果路径不够平滑, RRT重新初始化, 重新搜索
                        RRTnode_1.clear(); RRTnode_1.shrink_to_fit(); RRTnode_1.push_back({start[0], start[1]});
                        RRTnode_2.clear(); RRTnode_2.shrink_to_fit(); RRTnode_2.push_back({goal[0], goal[1]});

                        path_1.clear();        path_1.shrink_to_fit();
                        path_2.clear();        path_2.shrink_to_fit();
                        path.clear();          path.shrink_to_fit();
                        pruning_path.clear();  pruning_path.shrink_to_fit();
                        
                        child_to_parent_1.clear();child_to_parent_1.shrink_to_fit();child_to_parent_1.push_back(-1);
                        child_to_parent_2.clear();child_to_parent_2.shrink_to_fit();child_to_parent_2.push_back(-1);
                        continue;
                    }
                    else
                        success = 1;
                    
                    if (pruning_path[0] == goal){
                        reverse(pruning_path.begin(), pruning_path.end());
                    }
                    // break -> return main()
                    break;
                }
            }
        }
    }
    return pruning_path;
}


void B_RRT::set_boundary(float min_x, float min_y, float max_x, float max_y){
    minx = min_x;
    maxx = max_x;
    miny = min_y;
    maxy = max_y;
}

void B_RRT::set_avoid_dist(float dist){
    avoid_dist = dist;
}



point_t B_RRT::steer(point_t pt_near, point_t pt_sample){
    point_t pt_new={0,0};
    if(hypot(pt_near[0]-pt_sample[0], pt_near[1]-pt_sample[1]) < steer_dist)
        pt_new = pt_sample;
    
    else{
        double theta = atan2(pt_sample[1] - pt_near[1], pt_sample[0] - pt_near[0]);
        pt_new[0] = pt_near[0] + steer_dist * cos(theta);
        pt_new[1] = pt_near[1] + steer_dist * sin(theta);
    }
    return pt_new;
}


bool B_RRT::check_obs(point_t pt_near, point_t pt_new, KDTree &obstree){
    point_t pt = pt_near;
    double dx = pt_new[0] - pt_near[0];
    double dy = pt_new[1] - pt_near[1];
    double angle = atan2(dy, dx);
    double dis = hypot(dx, dy);
    pointIndex res;
    double nearest_dis;

    // step_size = robot_size + avoid_dist
    double step_size = 0.1;
    // steps = round(dis/step_size)
    int steps = int(dis/step_size);
    for(int i=0; i<steps; i++){
        res = obstree.nearest_pointIndex(pt);
        nearest_dis = hypot(pt[0]-res.first[0], pt[1]-res.first[1]);
        if (nearest_dis <= robot_size + avoid_dist)
            return true;
        pt[0] += step_size * cos(angle);
        pt[1] += step_size * sin(angle);
    }

    // 单独检查目标点
    res = obstree.nearest_pointIndex(pt_new);
    nearest_dis = hypot(pt_new[0]-res.first[0], pt_new[1]-res.first[1]);
    if (nearest_dis <= robot_size + avoid_dist)
        return true;
    return false;
}


pointVec B_RRT::find_path(pointVec &RRTnode, indexArr &child_to_parent){
    pointVec path;
    path.push_back(RRTnode[RRTnode.size()-1]);
    size_t parent = child_to_parent[child_to_parent.size()-1];
    while (parent != 0){
        path.push_back(RRTnode[parent]);
        parent = child_to_parent[parent];
    }
    path.push_back(RRTnode[0]);
    return path;
}


pointVec B_RRT::pruning(pointVec &path, KDTree &obstree){
    pointVec pruning_path;
    pruning_path.push_back(path[0]);
    point_t temp_point = path[0];
    for (int i=2; i<path.size(); i++){
        if (check_obs(temp_point, path[i], obstree)){
            pruning_path.push_back(path[i-1]);
            temp_point = path[i-1];
        }
    }
    pruning_path.push_back(path[path.size()-1]);
    return pruning_path;
}
    
    
double B_RRT::min_path_theta(pointVec &path){
    if (path.size() == 0)
        return 0;
    double min_path_theta = M_PI;
    for (int i=0; i<path.size()-2; i++){
        // 用余弦定理计算路径点夹角
        double a = hypot(path[i][0]-path[i+1][0], path[i][1]-path[i+1][1]);
        double b = hypot(path[i+1][0]-path[i+2][0], path[i+1][1]-path[i+2][1]);
        double c = hypot(path[i][0]-path[i+2][0], path[i][1]-path[i+2][1]);
        double cosine = (pow(a,2)+pow(b,2)-pow(c,2)) / (2*a*b);
        if (cosine < -1)
            cosine = -1;  // 避免错误
        double path_theta = acos(cosine);
        if (path_theta < min_path_theta)
            min_path_theta = path_theta;
    }
    return min_path_theta;
}
