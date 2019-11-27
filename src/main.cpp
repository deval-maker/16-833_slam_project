#include <utils.h>
#include <node.h>
#include <graph.h>
#include <search.h>
#include <MapReader.h>
#include <belief.h>
#include <information_gain.h>
#include <test.h>
using namespace std;

bool test = false;

int main()
{

    if(test) test_();
//------------- Map Reader  --------------------------------
    String map_path = "data/map2.txt";

    std::shared_ptr<MapReader> map_obj = std::make_shared<MapReader>(map_path);
    map_obj->visualize_map();
//------------- Uniqueness graph creation --------------------------------
    std::shared_ptr<Unique_Graph> unq_graph = std::make_shared<Unique_Graph>(map_obj, 500, 32);

    unq_graph->sample_vertices();
// ------------ Hardcode Modes --------------------------------------------
    Eigen::Vector3f firstmean, secondmean, thirdmean;
    firstmean << 150, 175, 0; 
    secondmean << 850, 175, 3.14;
    thirdmean <<  850, 850, 3.14/2;

    Eigen::Matrix3f sigma;
    sigma << 0.01, 0,    0,
             0,    0.01, 0,
             0,    0,    0.01;

    double weight = 1;

    mode firstMode(firstmean, sigma, weight);
    mode secondMode(secondmean, sigma, weight);
    mode thirdMode(thirdmean, sigma, weight);

// -------------Target state computation ------------------
    node firstNode (1, firstmean[0], firstmean[1], firstmean[2]);
    node secondNode (2, secondmean[0], secondmean[1], secondmean[2]);
    node thirdNode (3, thirdmean[0], thirdmean[1], thirdmean[2]);

    vector<node> Nodes{firstNode,secondNode, thirdNode}; 

    node firstTarget = unq_graph->target_state(firstNode, Nodes);
    node secondTarget = unq_graph->target_state(secondNode, Nodes);
    node thirdTarget = unq_graph->target_state(thirdNode, Nodes);

    point_t firstModePoint{firstNode.x, firstNode.y}, firstTargetPoint{firstTarget.x, firstTarget.y};
    point_t secondModePoint{secondNode.x, secondNode.y}, secondTargetPoint{secondTarget.x, secondTarget.y};
    point_t thirdModePoint{thirdNode.x, thirdNode.y}, thirdTargetPoint{thirdTarget.x, thirdTarget.y};
 
    // map_obj->visualize_point(firstModePoint);
    // map_obj->visualize_point(secondModePoint);
    // map_obj->visualize_point(thirdModePoint);

    // map_obj->visualize_point(firstTargetPoint);
    // map_obj->visualize_point(secondTargetPoint);
    // map_obj->visualize_point(thirdTargetPoint);
// -----------------------Plan to Target State ------------------------

    Search search;
    search.set_start(firstModePoint);
    search.set_goal(firstTargetPoint);
    search.set_mapReader(map_obj.get());
    vector<point_t> plan = search.plan();

    for(int i = 0; i < plan.size(); i++)
    {
        std::cout<<plan[i][0]<<' '<<plan[i][1]<<'\n';
    } 

    // Spawn Modes

    // Hardcode true Mode

    // Raycast and get all the measurements
}
