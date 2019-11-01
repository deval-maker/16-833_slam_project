#pragma once

#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


#define MAP_SIZE_X 800
#define MAP_SIZE_Y 800

class MapReader
{

public:
    ifstream infile;
    int size_x = MAP_SIZE_X;
    int size_y = MAP_SIZE_Y;

    uint8_t _viz_map[MAP_SIZE_X][MAP_SIZE_Y];
    float _map[MAP_SIZE_X][MAP_SIZE_Y];

    cv::Mat A = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8U);
    
    MapReader(string src_path_map);
    
    void visualize_map(void);
    float* get_prob_map(void);
    float query_map(int i, int j);
    uint8_t* get_viz_map(void);
};