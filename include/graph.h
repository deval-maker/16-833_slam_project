#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <random>
#include <math.h>
#include <functional>
#include <node.h>
#include <KDTree.hpp>
#include <bits/stdc++.h>



class point_hasher{
    public:

    size_t operator()(const point_t& pt) const
    {
        std::string str = to_string(pt[0])+","+to_string(pt[1])+","+to_string(pt[2]);
        return std::hash<string>()(str);
    }
};

class Unique_Graph
{
    public:
    int num_vertices;
    int x_size;
    int y_size;
    std::vector<node> vertices;
    pointVec points;
    KDTree knn_tree;
    std::vector<std::vector<int>> adjacency_mat;
    std::unordered_map<point_t,node,point_hasher> node_map;
    void sample_vertices();

    double m_maxTargetDist;

    Unique_Graph()
    {

    }

    Unique_Graph(int num_vertices, int x_size, int y_size, int num_headings)
    {
        this->num_vertices = num_vertices;
        this->x_size = x_size;
        this->y_size = y_size;

        for(double i = 0; i< M_2_PI; i += M_2_PI/num_headings){

            discrete_headings.push_back(i);

        }


    }

    private:

    vector<double> discrete_headings;

    void create_adj_mat();
    static int similarity( node& node1, node& node2);
    bool check_dist(node mode, node vertex);
    node target_state(node targetMode, std::vector<node> modes);
};
