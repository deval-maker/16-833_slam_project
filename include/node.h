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
    friend ostream& operator<<(ostream& os, const node& dt);
    static int similarity(node &node1, node &node2);
    
};