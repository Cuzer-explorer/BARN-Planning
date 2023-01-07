#include <vector>

using namespace std;

using point_t = vector< double >;
using indexArr = vector< size_t >;
using pointVec = vector< point_t >;

class DWA{
private:
    // 定义机器人移动极限速度、加速度等运动学约束信息
    double v;
    double w_max;
    double wa;
    double w_reso;
    double safe_dist;
    double penalty;  // 距离障碍物过近的惩罚项
    double dt;  // 0.1
    double predict_time;
    double delay_predict_time;
    double goal_cost_gain;
    double dist_cost_gain;  // 
    double global_path_cost_gain;


public:
    void load_param(); // 加载参数

    // 设定线速度
    void set_v(double expected_v);

    // 速度窗口
    vector<double> dynamic_window(double w);

    // 模拟一条轨迹中位置、速度的计算
    vector<double> motion_model(vector<double> x, double w, double delta_t);

    // 进行一条轨迹的预测
    vector<vector<double>> predict_trajectory(vector<double> x, double w, double predict_time);

    // 目标代价
    double goal_cost(vector<vector<double>> &traj, point_t &goal);

    // 障碍物距离代价
    double distance_cost(vector<vector<double>> &traj, pointVec &obstacle);

    // 与全局路径距离代价
    // double global_path_cost(vector<vector<double>> &traj, pointVec &path);

    // 轨迹选择
    pair<double, vector<vector<double>>>  control_trajectory(vector<double> &x,
            double &w, point_t &goal, pointVec &obstacle, int delay);
};