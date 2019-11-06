#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <random>
#include <math.h>
#include <functional>
#include <node.h>
#include <graph.h>
#include <memory>
#include <search.h>


using namespace std;

int main()
{
    std::cout << "Hello Project!" << std::endl;

    std::shared_ptr<Unique_Graph> unq_graph = std::make_shared<Unique_Graph>(100,200,200,32);

    unq_graph->sample_vertices();
    auto pt = unq_graph->knn_tree.nearest_point({1,2,3});

    auto near_node = unq_graph->node_map[pt];


}
