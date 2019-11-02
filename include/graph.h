#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <random>
#include <math.h>
#include <functional>
#include <node.h>

class Unique_Graph
{
    public:
    int num_vertices;
    int x_size;
    int y_size;
    std::vector<node> vertices;
    std::vector<std::vector<double>> adjacency_mat; 

    Unique_Graph()
    {

    }
    
    Unique_Graph(int num_vertices, int x_size, int y_size)
    {
        this->num_vertices = num_vertices;
        this->x_size = x_size;
        this->y_size = y_size;
    }

    private:

    void sample_vertices();
    void create_adj_mat();
    int similarity( node& node1, node& node2);
    
};