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

    point_t point_to_viz;
    point_to_viz.push_back(100.0);
    point_to_viz.push_back(200.0);
    point_to_viz.push_back(0.0);
    map_obj->visualize_point(point_to_viz);

    // Spawn Modes

    // Hardcode true Mode

    // Raycast and get all the measurements
}
