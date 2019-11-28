#include <utils.h>
#include <node.h>
#include <graph.h>
#include <search.h>
#include <MapReader.h>
#include <belief.h>
#include <information_gain.h>
#include <spawn_modes.h>
#include <test.h>
using namespace std;

bool test = true;

vector<point_t> convert_to_path(point_t start_state, vector<point_t> actions)
{
    vector<point_t> plan;
    plan.push_back(start_state);
    point_t current = start_state;
    for(int i = actions.size() - 1; i >= 0; i--)
    {
        current[0] += actions[i][0];
        current[1] += actions[i][1];
        // std::cout<<"Actions "<<actions[i][0]<<' '<<actions[i][1]<<'\n';
        plan.push_back(current);
    }
    return plan;
}

void test_spawn_modes()
{
  std::cout<<"Testing spawning modes \n";
  String map_path = "data/map4.txt";
  MapReader map_obj = MapReader(map_path);
  map_obj.visualize_map();

  Eigen::Matrix3f sigma_init;
  sigma_init << 0.01, 0,    0,
                0,    0.01, 0,
                0,    0,    0.01;
  int modes_num = 50;

  Eigen::Vector3f original_mean(176,85,0);
  mode original_mode(original_mean, sigma_init, 1);

  vector<mode> spawned_modes = spawn_modes(original_mode, sigma_init, modes_num, &map_obj);
}

int main()
{

    if(test)
    {
        test_spawn_modes();
        return 0;
    }

//------------- Map Reader  --------------------------------
    String map_path = "data/map4.txt";

    std::shared_ptr<MapReader> map_obj = std::make_shared<MapReader>(map_path);
    map_obj->visualize_map();

//------------- Uniqueness graph creation --------------------------------
    std::shared_ptr<Unique_Graph> unq_graph = std::make_shared<Unique_Graph>(map_obj, 2000, 32);

    unq_graph->sample_vertices();
    unq_graph->viz_graph();
// ------------------------Spawn Modes -----------------------------------

// ------------ Hardcode Modes --------------------------------------------
    Eigen::Vector3f firstmean, secondmean, thirdmean;
    firstmean << 225, 605, 0;
    secondmean << 850, 175, 3.14;
    thirdmean <<  640, 830, 3.14/2;

    Eigen::Matrix3f sigma;
    sigma << 0.01, 0,    0,
             0,    0.01, 0,
             0,    0,    0.01;

    double weight = 1;

    mode firstMode(firstmean, sigma, weight);
    firstMode.visualize_ellipse(map_obj.get());
    mode secondMode(secondmean, sigma, weight);
    mode thirdMode(thirdmean, sigma, weight);

    vector<mode> modes{firstMode, secondMode, thirdMode};

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

    vector<point_t> visualizationPoint{firstModePoint, firstTargetPoint, secondModePoint,
    secondTargetPoint, thirdModePoint, thirdTargetPoint};


    // map_obj->visualize_point(firstModePoint);
    // map_obj->visualize_point(secondModePoint);
    // map_obj->visualize_point(thirdModePoint);

    // map_obj->visualize_point(firstTargetPoint);
    // map_obj->visualize_point(secondTargetPoint);
    // map_obj->visualize_point(thirdTargetPoint);
// -----------------------Plan to Target State ------------------------

    vector<vector<point_t>> plans;
    for(int counter = 0; counter < 3; counter++)
    {
        point_t start_point, goal_point;
        Search search;
        if(counter == 0)
        {
            start_point = firstModePoint;
            goal_point = firstTargetPoint;
        }
        else if(counter == 1)
        {
            start_point = secondModePoint;
            goal_point = secondTargetPoint;
        }
        else
        {
            start_point = thirdModePoint;
            goal_point = thirdTargetPoint;
        }
        search.set_start(start_point);
        search.set_goal(goal_point);
        search.set_mapReader(map_obj.get());
        vector<point_t> actions = search.plan();
        // std::cout<<"actions size "<<actions.size()<<'\n';
        vector<point_t> plan = convert_to_path(start_point, actions);
        std::cout<<"plan size "<<plan.size()<<'\n';

        map_obj->visualize_path(plan, cv::viz::Color::green());
        plans.push_back(actions);
    }

    for(int i = 0; i< plans.size(); i++)
    {
        reverse(plans[i].begin(), plans[i].end());
    }
    map_obj->visualize_path(visualizationPoint, cv::viz::Color::red());
// -----------------------Pick Optimal Policy ------------------------

    // get_optimal_policy(plans, modes, map_obj.get());
    // point_t start_point{100, 400}, goal_point{200, 600};
    // vector<point_t> actions = get_control(start_point, goal_point, map_obj.get());

    // for(int i = 0; i< actions.size(); i++)
    // {
    //     std::cout<<actions[i][0]<<" "<<actions[i][1]<<'\n';
    // }

    get_optimal_policy(plans,modes,map_obj.get());

// ----------------------- Visualisation  ------------------------
    map_obj->viz_session(); //  Creating a session for visualisation



}
