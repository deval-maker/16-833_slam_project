#pragma once

#include <utils.h>
#include <node.h>
#include <eigen3/Eigen/Dense>

#define MAP_SIZE_X 1000
#define MAP_SIZE_Y 1000

class MapReader
{

private:
    int total_landmarks = 254;
    static const int stepsize = 1;
    double delta_theta = stepsize * M_PI/180.0;
    float delta_dist = 1;
    float laser_fov = 360;
    vector<point_t> landmark_lookup;

    // images
    cv::Mat A;
    cv::Mat B;

public:
    ifstream infile;
    int size_x = MAP_SIZE_X;
    int size_y = MAP_SIZE_Y;

    uint8_t _map[MAP_SIZE_X][MAP_SIZE_Y];
    
    MapReader(string src_path_map);
    
    void visualize_map(void);
    void visualize_point(point_t point, cv::viz::Color color);
    void visualize_point_and_dir(point_t point, cv::viz::Color color);
    void visualize_path(vector<point_t> path, cv::viz::Color color);
    void visualize_UG(vector<node> ug, cv::viz::Color color);
    void visualize_ellipse(Eigen::Vector2f mean, Eigen::Matrix2f sigma);

    void clear_session();
    void viz_session();

    uint8_t query_map(int i, int j);
    uint8_t* get_map(void);

    void update_visible_landmarks(node &x_t, bool visualize);
    vector<meas> get_landmark_measurement(Eigen::Vector3f curr_pose);
    point_t get_landmark_pose(int landmark_id);

    double correct_range(double angle);
    float toRadian(float degree);
    
};