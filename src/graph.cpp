#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <graph.h>
#include <random>
#include <math.h>
#include <functional>
#include <node.h>
#include <bits/stdc++.h>

void Unique_Graph::sample_vertices()
{
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution_x(0, x_size);
    std::uniform_int_distribution<int> distribution_y(0, y_size);
    std::uniform_real_distribution<float> distribution_theta_idx(0, discrete_headings.size());

    auto x = std::bind(distribution_x,generator);
    auto y = std::bind(distribution_y,generator);
    auto theta_idx = std::bind(distribution_theta_idx,generator);

    for (int i=0; i< num_vertices; i++){

        int sample_x = x();
        int sample_y = y();
        int sample_theta = discrete_headings[theta_idx()];
        point_t pt = {sample_x,sample_y,sample_theta};
        points.push_back(pt);
        node sample_node(i,sample_x,sample_y,sample_theta);
        node_map.insert(std::make_pair(pt,sample_node));
        vertices.push_back(sample_node);

    }

    knn_tree = KDTree(points);
    return;
}


void Unique_Graph::create_adj_mat()
{
    for(int i =0; i<num_vertices; i++){
        for(int j=0; j<num_vertices; j++){
            if(i != j)
            {
                int sim = node::similarity(vertices[i],vertices[j]);
                adjacency_mat[vertices[i].id][vertices[j].id] = sim;
                adjacency_mat[vertices[j].id][vertices[i].id] = sim;

            }
            else
            {
                adjacency_mat[vertices[i].id][vertices[j].id] = 0;

            }

        }
    }
}

bool Unique_Graph::check_dist(node mode, node vertex)
{
  return pow(pow((mode.x - vertex.x),2) + pow(mode.y-vertex.y,2),1.0/2) < m_maxTargetDist;
}

node Unique_Graph::target_state(node targetMode, std::vector<node> modes)
{

  point_t targetModePoint{double(targetMode.x),double(targetMode.y),double(targetMode.theta)};
  pointVec targetNeighborPoints = knn_tree.neighborhood_points(targetModePoint,m_maxTargetDist);

  int minWeight = INT_MAX;
  node target_state;

  for(int i = 0; i < targetNeighborPoints.size();i++)
  {
    node targetNeighbor = node_map[targetNeighborPoints[i]];
    double weight = 0;
    for(int j=0;j<modes.size();j++)
    {
      if(modes[j].id == targetMode.id) continue;
      point_t otherModePoint{double(modes[j].x), double(modes[j].y), double(modes[j].theta)};
      pointVec otherNeighborPoints = knn_tree.neighborhood_points(otherModePoint,m_maxTargetDist);

      for(int k = 0; k<otherNeighborPoints.size();k++)
      {
        node otherNeighbor = node_map[otherNeighborPoints[k]];
        weight+=adjacency_mat[otherNeighbor.id][targetNeighbor.id];
      }

    }
    if(weight < minWeight)
    {
      minWeight = weight;
      target_state = targetNeighbor;
    }
  }

  return target_state;
}
