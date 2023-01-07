#include <vector>
#include "KDTree.hpp"
#include <cmath>


using namespace std;

using point_t = vector< double >;
using indexArr = vector< size_t >;
using pointVec = vector< point_t >;


class B_RRT{ // 双向RRT

private:
    int N_SAMPLE;
    float minx;
    float maxx;
    float miny;
    float maxy;
    float robot_size;
    float avoid_dist;
    float steer_dist;
    // float goal_error = 0.2;
    float allow_path_theta;

public:
    void load_param();
    pointVec plan(pointVec &obstacle, point_t start, point_t goal);
    void set_boundary(float min_x, float min_y, float max_x, float max_y);
    void set_avoid_dist(float dist);
    point_t steer(point_t pt_near, point_t pt_sample);
    bool check_obs(point_t pt_near, point_t pt_new, KDTree &obstree);
    pointVec find_path(pointVec &RRTnode, indexArr &child_to_parent);
    pointVec pruning(pointVec &path, KDTree &obstree);
    double min_path_theta(pointVec &path);
};