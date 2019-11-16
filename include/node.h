#pragma once

#include <utils.h>

class node
{
    public:
    int id;
    int x;
    int y;
    double theta;
    std::unordered_set<int> visible_landmarks;

    node()
    {

    }

    node(int id, int x, int y, double theta);
    
    static int similarity(node &node1, node &node2);
    
};