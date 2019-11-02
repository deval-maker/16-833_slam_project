#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <random>
#include <math.h>
#include <functional>
#include <bits/stdc++.h>

class node
{
    public:
    int id;
    int x;
    int y;
    int theta;
    std::unordered_set<int> visible_landmarks;

    node()
    {

    }

    node(int id, int x, int y, int theta);
    
    void add_visible_landmarks(MapReader &map_obj);
    static int similarity(node &node1, node &node2);
    
};