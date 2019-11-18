#include <utils.h>
#include <node.h>
#include <graph.h>
#include <search.h>
#include <MapReader.h>
#include <test.h>

using namespace std;

bool test = true;

int main()
{
    if(test)
    {
      test_raycast1();
      test_raycast2();

      test_controller1();
      test_controller2();
      test_controller3();
      test_controller4();
    }
    
    String map_path = "data/map1.txt";
    MapReader map_obj = MapReader(map_path);
 
    std::shared_ptr<Unique_Graph> unq_graph = std::make_shared<Unique_Graph>(100,200,200,32);

    unq_graph->sample_vertices();
    auto pt = unq_graph->knn_tree.nearest_point({1,2,3});

    auto near_node = unq_graph->node_map[pt];

    Search search;
    point_t start{1,4};
    point_t goal{10,10};
    search.set_start(start);
    search.set_goal(goal);

    std::vector<point_t> path = search.plan();
    std::cout<<"Path \n";
    for(int i=0;i<path.size();i++)
    {
      for(int j=0;j<path[i].size();j++)
      {
        std::cout<<path[i][j]<<' ';
      }
      std::cout<<'\n';
    }
}
