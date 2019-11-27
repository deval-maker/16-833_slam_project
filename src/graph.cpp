
#include <graph.h>
#include <MapReader.h>

void Unique_Graph::sample_vertices()
{
    srand(time(0));
    std::random_device dev;
    std:: mt19937 rng(dev());
    std::uniform_int_distribution<int> distribution_x(1, x_size-1);
    std::uniform_int_distribution<int> distribution_y(1, y_size-1);
    std::uniform_int_distribution<int> distribution_theta_idx(0, discrete_headings.size());



    int sampled_vertices = 0;
    while (sampled_vertices < num_vertices){

        int sample_x = distribution_x(rng);
        int sample_y = distribution_y(rng);

        if(map->query_map(sample_y,sample_x) <= 10){
          continue;
        }

        int sample_theta = discrete_headings[distribution_theta_idx(rng)];
        point_t pt = {sample_x,sample_y,sample_theta};
        points.push_back(pt);
        node sample_node(sampled_vertices,sample_x,sample_y,sample_theta);
        map->update_visible_landmarks(sample_node,0);
        node_map.insert(std::make_pair(pt,sample_node));
        vertices.push_back(sample_node);
        sampled_vertices++;

    }

    knn_tree = KDTree(points);
    std::cout<<"[INFO] Finished Sampling "<<num_vertices<<" Graph Vertices"<<std::endl;


    create_adj_mat();
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
  std::cout<<"[INFO] Generated the Adjacency Matrix"<<std::endl;
}

bool Unique_Graph::check_dist(node mode, node vertex)
{
  return pow(pow((mode.x - vertex.x),2) + pow(mode.y-vertex.y,2),1.0/2) < m_maxTargetDist;
}


void::Unique_Graph::viz_graph()
{

  map->visualize_UG(vertices,cv::viz::Color::celestial_blue());
}

node Unique_Graph::target_state(node targetMode, std::vector<node> modes)
{

  point_t targetModePoint{double(targetMode.x),double(targetMode.y),double(targetMode.theta)};
  pointVec targetNeighborPoints = knn_tree.neighborhood_points(targetModePoint,m_maxTargetDist);
  
  map->visualize_path(targetNeighborPoints, cv::viz::Color::yellow());
  
  // std::cout<<"Target Neighbor points size "<<targetNeighborPoints.size()<<'\n';

  // visualize_nodes(vertices);
  // visualize_nodes(targetNeighborPoints);
  int minWeight = INT_MAX;
  node target_state;

  for(int i = 0; i < targetNeighborPoints.size();i++)
  {
    node targetNeighbor = node_map[targetNeighborPoints[i]];
    double weight = 0;
    // std::cout<<"Total number of modes "<<modes.size()<<'\n';
    for(int j=0;j<modes.size();j++)
    {
      if(modes[j].id == targetMode.id) continue;
      point_t otherModePoint{double(modes[j].x), double(modes[j].y), double(modes[j].theta)};
      pointVec otherNeighborPoints = knn_tree.neighborhood_points(otherModePoint,m_maxTargetDist);
      // std::cout<<"Other Neighbor points size "<<otherNeighborPoints.size()<<'\n';

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
