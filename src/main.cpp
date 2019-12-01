#include <graph.h>
#include <utils.h>
#include <node.h>
#include <search.h>
#include <MapReader.h>
#include <belief.h>
#include <information_gain.h>
#include <spawn_modes.h>
#include <GMM_update.h>
#include <test.h>
using namespace std;

bool test = false;

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
  String map_path = "data/map5.txt";
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
    String map_path = "data/map_dummy.txt";

    std::shared_ptr<MapReader> map_obj = std::make_shared<MapReader>(map_path);
    map_obj->visualize_map();

//------------- Uniqueness graph creation --------------------------------
    std::shared_ptr<Unique_Graph> unq_graph = std::make_shared<Unique_Graph>(map_obj, 1000, 32);

    unq_graph->sample_vertices();
    // unq_graph->viz_graph();

// ------------ Hardcode Modes --------------------------------------------
    Eigen::Vector3f firstmean, secondmean, thirdmean, fourthmean;
    firstmean << 150, 200, -M_PI/2;
    secondmean << 257, 800, M_PI/2;
    thirdmean <<  855, 200, -M_PI/2;
    fourthmean << 855,800, M_PI/2;

    Eigen::Matrix3f sigma;
    sigma << 0.01, 0,    0,
             0,    0.01, 0,
             0,    0,    0.01;

    double weight = 0.25;

    mode firstMode(firstmean, sigma, weight);
    // firstMode.visualize_ellipse(map_obj.get());
    mode secondMode(secondmean, sigma, weight);
    mode thirdMode(thirdmean, sigma, weight);
    mode fourthMode(fourthmean, sigma, weight);

    vector<mode> modes{firstMode, secondMode, thirdMode,fourthMode};
// ----------------------Spawn Ground Truth -----------------------------------

    mode groundTruth(firstmean, sigma, weight);

// ------------------------Spawn Modes -----------------------------------
  int modes_num = 5;

  vector<mode> spawned_modes = spawn_modes(groundTruth, sigma, modes_num, map_obj.get());
  vector<node> spawnedNodes;

   for(int i = 0; i < spawned_modes.size(); i++)
    {
        node i_node(i+1, spawned_modes[i].mean[0], spawned_modes[i].mean[1], 
        spawned_modes[i].mean[2]);
        spawnedNodes.push_back(i_node);
        point_t modePoint{i_node.x, i_node.y, i_node.theta};
        map_obj->visualize_point_and_dir(modePoint, cv::viz::Color::red());
    }
    std::cout<<"Spawned modes printed \n";
    std::cout<<"Number of spawned modes "<<spawned_modes.size()<<'\n';
    map_obj->viz_session();

// -------------Target state computation ------------------
    vector<node> Targets;
    for(int i = 0; i < spawnedNodes.size(); i++)
    {
        node Target = unq_graph->target_state(spawnedNodes[i], spawnedNodes);
        Targets.push_back(Target);
        point_t targetPoint{Target.x, Target.y, Target.theta};
        map_obj->visualize_point_and_dir(targetPoint, cv::viz::Color::green());
    }
    std::cout<<"Visualising start and goal points \n";
    map_obj->viz_session();
    return 0;
    node firstNode (1, firstmean[0], firstmean[1], firstmean[2]);
    node secondNode (2, secondmean[0], secondmean[1], secondmean[2]);
    node thirdNode (3, thirdmean[0], thirdmean[1], thirdmean[2]);
    node fourthNode(4,fourthmean[0], fourthmean[1], fourthmean[2]);

    vector<node> Nodes{firstNode,secondNode, thirdNode, fourthNode};

    node firstTarget = unq_graph->target_state(firstNode, Nodes);
    node secondTarget = unq_graph->target_state(secondNode, Nodes);
    node thirdTarget = unq_graph->target_state(thirdNode, Nodes);
    node fourthTarget = unq_graph->target_state(fourthNode, Nodes);
    

    point_t firstModePoint{firstNode.x, firstNode.y, firstNode.theta}, firstTargetPoint{firstTarget.x, firstTarget.y,firstTarget.theta};
    point_t secondModePoint{secondNode.x, secondNode.y, secondNode.theta}, secondTargetPoint{secondTarget.x, secondTarget.y, secondTarget.theta};
    point_t thirdModePoint{thirdNode.x, thirdNode.y, thirdNode.theta}, thirdTargetPoint{thirdTarget.x, thirdTarget.y, thirdTarget.theta};
    point_t fourthModePoint{fourthNode.x, fourthNode.y, fourthNode.theta}, fourthTargetPoint{fourthTarget.x, fourthTarget.y, fourthTarget.theta};


    vector<point_t> visualizationPoint{firstModePoint, firstTargetPoint, secondModePoint,
    secondTargetPoint, thirdModePoint, thirdTargetPoint, fourthModePoint, fourthTargetPoint};

    for(int i = 0; i < visualizationPoint.size(); i++)
    {
        map_obj->visualize_point_and_dir(visualizationPoint[i], cv::viz::Color::red());
    }
    // map_obj->update_visible_landmarks(firstNode,1);
    // map_obj->update_visible_landmarks(secondNode,1);
    // map_obj->update_visible_landmarks(thirdNode,1);

    // map_obj->visualize_point(firstTargetPoint);
    // map_obj->visualize_point(secondTargetPoint);
    // map_obj->visualize_point(thirdTargetPoint);
// -----------------------Plan to Target State ------------------------

    vector<vector<point_t>> plans;
    for(int counter = 0; counter < 4; counter++)
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
        else if (counter == 2)
        {
            start_point = thirdModePoint;
            goal_point = thirdTargetPoint;
        }
        else{
            start_point = fourthModePoint;
            goal_point = fourthTargetPoint;

        }
        search.set_start(start_point);
        search.set_goal(goal_point);
        search.set_mapReader(map_obj.get());
        vector<point_t> actions = search.plan();
        // std::cout<<"actions size "<<actions.size()<<'\n';
        vector<point_t> plan = convert_to_path(start_point, actions);
        // std::cout<<"plan size "<<plan.size()<<'\n';

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

    int optimal_policy_index = get_optimal_policy(plans,modes,map_obj.get());
// ----------------------- Actual propagation and GMM weight update  ------------------------
    propagate_policy(plans[optimal_policy_index], groundTruth, modes, map_obj.get());
    
// ----------------------- Visualisation  ------------------------
    map_obj->viz_session(); //  Creating a session for visualisation



}
