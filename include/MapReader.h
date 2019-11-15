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


#define MAP_SIZE_X 1000
#define MAP_SIZE_Y 1000

class MapReader
{

public:
    ifstream infile;
    int size_x = MAP_SIZE_X;
    int size_y = MAP_SIZE_Y;

    uint8_t _map[MAP_SIZE_X][MAP_SIZE_Y];

    cv::Mat A = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8U);
    
    MapReader(string src_path_map);
    
    void visualize_map(void);
    uint8_t query_map(int i, int j);
    uint8_t* get_map(void);
};