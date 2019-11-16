#pragma once

#include <utils.h>
#include <node.h>

#define MAP_SIZE_X 1000
#define MAP_SIZE_Y 1000

class MapReader
{

private:
    int total_landmarks = 10;
    static const int stepsize = 1;
    double delta_theta = stepsize * PI/180.0;
    float delta_dist = 1.0;
    float laser_fov = 360;

public:
    ifstream infile;
    int size_x = MAP_SIZE_X;
    int size_y = MAP_SIZE_Y;

    uint8_t _map[MAP_SIZE_X][MAP_SIZE_Y];
    
    MapReader(string src_path_map);
    
    void visualize_map(void);
    uint8_t query_map(int i, int j);
    uint8_t* get_map(void);

    void update_visible_landmarks(node *, bool);
    double correct_range(double angle);
    float toRadian(float degree);

};