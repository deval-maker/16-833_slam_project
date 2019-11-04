#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <graph.h>
#include <random>
#include <math.h>
#include <functional>
#include <node.h>

void Unique_Graph::sample_vertices()
{
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution_x(0, x_size);
    std::uniform_int_distribution<int> distribution_y(0, y_size);
    std::uniform_real_distribution<float> distribution_theta(-3.14, 3.14);

    auto x = std::bind(distribution_x,generator);
    auto y = std::bind(distribution_y,generator);
    auto theta = std::bind(distribution_theta,generator);

    for (int i=0; i< num_vertices; i++){

        node sample_node(i,x(),y(),theta());
        vertices.push_back(sample_node);

    }

    return;
}


void Unique_Graph::create_adj_mat()
{
    for(int i =0; i<num_vertices; i++){
        for(int j=0; j<num_vertices; j++){
            if(i != j)
            {
                int sim = similarity(vertices[i],vertices[j]);
                adjacency_mat[vertices[i].id,vertices[j].id] = sim;
                adjacency_mat[vertices[j].id,vertices[i].id] = sim;

            }
            else
            {
                adjacency_mat[vertices[i].id,vertices[j].id] = 0;

            }

        }
    }
}
bool check_dist(node mode, node vertex)
{
  return pow(pow((mode.x - vertex.x),2) + pow(mode.y-vertex.y,2),1.0/2) < m_maxTargetDist;
}

node Unique_Graph::target_state(node mode)
{
  int similarity = 100;
  node target_state;
  for(int i=0;i<vertices.size();i++)
  {
    if(mode.id!=vertices[i].id && check_dist(mode,vertices[i]))
    {
      if(adjacency_mat[mode.id,vertices[i].id] < similarity)
      {
        similarity = adjacency_mat[mode.id,vertices[i].id];
        target_state = vertices[i];
      }
    }
  }
  return target_state;
}
