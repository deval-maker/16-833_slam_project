#pragma once

#include <utils.h>
#include <node.h>
#include <MapReader.h>
#include <memory>
#include <vector>
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
    std::shared_ptr<MapReader> map;
    std::vector<node> vertices;
    pointVec points;
    KDTree knn_tree;
    int a =10;

    std::vector<std::vector<int>> adjacency_mat;
    //int adjacency_mat[num_vertices][num_vertices];

    //std::array<std::array<int,num_vertices>,num_vertices> adjacency_mat;
    std::unordered_map<point_t,node,point_hasher> node_map;
    void sample_vertices();

    double m_maxTargetDist;

    Unique_Graph()
    {

    }

    Unique_Graph(shared_ptr<MapReader> map,int num_vertices, int num_headings)
    {   
        this->map = map;
        this->num_vertices = num_vertices;
        this->x_size = map->size_x;
        this->y_size = map->size_y;
        std::vector<std::vector<int> > mat(num_vertices,std::vector<int>(num_vertices));
        adjacency_mat = mat;

        for(double i = 0; i< 2*M_PI; i += 2*M_PI/num_headings){
            
            
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
