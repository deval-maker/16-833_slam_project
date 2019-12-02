#pragma once
#include <utils.h>
#include <controller.h>

vector<point_t> get_control(point_t start_state, point_t goal_state, MapReader* map_obj)
{
    vector<point_t> controls;
    point_t ret;
    velocities steer_velos;
    node n1 = node(1,start_state[0],start_state[1],start_state[2]);

    Controller c1 = Controller();
    c1.set_current_state(n1);
    c1.set_goal(goal_state[0], goal_state[1], goal_state[2]);
    c1.drive_type = Controller::Steer;
    bool is_goal_reached = false;
    auto start = std::chrono::high_resolution_clock::now();
    double time_elapsed = 0.0;


    while(!is_goal_reached)
    {

        point_t step_vels;
        is_goal_reached = c1.next_time_step(steer_velos);

        c1.set_node_state(n1);
        // std::cout<<"start"<<start_state[0]<<" "<<start_state[1]<<" "<<start_state[2]<<"\n";
        // std::cout<<"goal"<<goal_state[0]<<" "<<goal_state[1]<<" "<<goal_state[2]<<"\n";
        // std::cout<<"curr_state"<<"\n";
        // std::cout<<n1<<std::endl;
        step_vels.push_back(steer_velos.linx);
        step_vels.push_back(steer_velos.ang);
        controls.push_back(step_vels);

        // map_obj->update_visible_landmarks(n1, true);


    }


    return controls;
}
vector<point_t> convert_to_path_information(point_t start_state, vector<point_t> actions, double relative_angle)
{
    vector<point_t> plan;
    plan.push_back(start_state);
    point_t current = start_state;
    for(int i = 0; i < actions.size(); i++)
    {
        current[0] += cos(relative_angle)*actions[i][0] - sin(relative_angle)*actions[i][1];
        current[1] += sin(relative_angle)*actions[i][0] + cos(relative_angle)*actions[i][1];
        // std::cout<<"Actions "<<actions[i][0]<<' '<<actions[i][1]<<'\n';
        plan.push_back(current);
    } 
    return plan;
}


int get_optimal_policy(vector<vector<point_t>> paths, vector<mode> modes,
MapReader* _map)
{
    std::cout<<"[INFO] Choosing the optimal policy \n";
    double wt_threshold = exp(-2);
    std::vector<double> information_gains;
    double collision_penalty = 100;  
    for(int i = 0; i < paths.size(); i++)
    {
        std::cout<<"[INFO] Simulating Path: "<<i<<'\n';
        double information_gain_policy = 0;

        std::vector<std::vector<point_t>> plans;

        for(int m = 0; m< modes.size(); m++)
        {
            double relative_angle = - modes[m].mean[2] + modes[i].mean[2];
            point_t start{modes[m].mean[0], modes[m].mean[1]};
            plans.push_back(convert_to_path_information(start,paths[i],relative_angle));
        }

        for(int j = 0; j < modes.size(); j++)
        {
            std::unordered_set<int> colliding_modes;

            std::cout<<" j th mode is "<<j<<'\n';
            int information_gain_mode = 0;

            vector<mode> modes_copy = modes;

            for(int t = 0; t < plans[i].size(); t++)
            {
                std::cout<<"Timstep "<<t<<'\n';
                double goal[2];

                point_t start_state, goal_state;
                start_state.push_back(modes_copy[j].mean[0]);
                start_state.push_back(modes_copy[j].mean[1]);
                start_state.push_back(modes_copy[j].mean[2]);

                goal_state.push_back(plans[j][t][0]);
                goal_state.push_back(plans[j][t][1]);

                std::cout<<"Before Get control \n";
                std::cout<<start_state[0]<<" "<<start_state[1]<<" "<<start_state[2]<<"\n";
                std::cout<<goal_state[0]<<" "<<goal_state[1]<<" "<<goal_state[2]<<"\n";

                if(start_state[0] >= 0 && start_state[1] >= 0)
                {
                    vector<point_t> controls = get_control(start_state, goal_state, _map);
                    std::cout<<"After Get control \n";

                    for(auto control: controls){
                        modes_copy[j].propagate_motion(control[0], control[1]);

                    }
                    std::cout<<"Propagated motion of jth mode \n";
                }
                // std::cout<<"Mean of j: "<<modes_copy[j].mean[0]<<" "<<modes_copy[j].mean[1]<<" "<<
                // modes_copy[j].mean[2]<<'\n';
                // std::cout<<"Visualizing j ellipse \n";
                // modes_copy[j].visualize_ellipse(_map);
                
                // std::cout<<"\n Visualizing k ellipses \n";

                Eigen::Vector3f pose_mode_j = modes_copy[j].mean;
                vector<meas> actual_meas = _map->get_landmark_measurement(pose_mode_j);
                modes_copy[j].update_weight(actual_meas,_map,t);

                for(int k = 0; k < modes_copy.size(); k++)
                {
                    if(k == j) continue;
                    if(colliding_modes.find(k) != colliding_modes.end()) continue; 
                //  
                    // std::cout<<" "<<"\n";
                    // std::cout<<"[DEBUG] Propogating mode "<<k<<std::endl;
                    point_t start_state, goal_state;
                    start_state.push_back(modes_copy[k].mean[0]);
                    start_state.push_back(modes_copy[k].mean[1]);
                    start_state.push_back(modes_copy[k].mean[2]);

                    goal_state.push_back(plans[k][t][0]);
                    goal_state.push_back(plans[k][t][1]);

                    std::cout<<start_state[0]<<" "<<start_state[1]<<" "<<start_state[2]<<"\n";
                    std::cout<<goal_state[0]<<" "<<goal_state[1]<<" "<<goal_state[2]<<"\n";

                    vector<point_t> controls = get_control(start_state, goal_state, _map);
                    std::cout<<"after get control2"<<"\n";
                //
                    for(auto control: controls){
                        modes_copy[k].propagate_motion(control[0], control[1]);
                
                    }
                    // modes_copy[k].visualize_ellipse(_map);
                    // modes_copy[k].update_measurement(actual_meas,_map);
                    // std::cout<<"Updating weight for Mode "<<k<<"\n";
                    modes_copy[k].update_weight(actual_meas, _map,t);
                    if(_map->query_map(modes_copy[k].mean[1], modes_copy[k].mean[0]) != 255)
                    {
                        colliding_modes.insert(k);
                        information_gain_policy -= double(collision_penalty)/t;
                        // std::cout<<"Current collision penalty "<<double(collision_penalty)/t;
                        // std::cout<<"Information gain policy"<<information_gain_policy<<'\n';
                    }
                    // std::cout<<"Weight for mode "<<k<<" "<<modes_copy[k].weight<<"\n";
                }

                std::cout<<"Propagated motion of the kth modes; Path length"<<plans[i].size()<< "\n";

                for(int i = 0; i < modes_copy.size(); i++)
                {
                    double weight_sum = 0;
                    for(int i = 0; i < modes_copy.size(); i++)
                        weight_sum += modes_copy[i].weight; 
                    
                    modes_copy[i].weight /= weight_sum;

                    std::cout<<"Intermediate weight of mode : "<<i<<" "<<modes_copy[i].weight<<'\n';
                }

                // _map->viz_session();

            }

            for(int k = 0; k < modes_copy.size(); k++)
            {
                std::cout<<"[DEBUG] Weight for Mode "<<k<<" is "<<modes_copy[k].weight<<std::endl;
                if(modes_copy[k].weight <= wt_threshold)
                    information_gain_mode++;
                
            }
            information_gain_policy += information_gain_mode;

        }
        information_gains.push_back(information_gain_policy);
    }
    _map->viz_session();

    for(int i = 0; i<information_gains.size(); i++)
    {
    std::cout<<"[INFO] Information Gain for Policy "<<i<<" is "<<information_gains[i]<<'\n';

    }
    return distance(information_gains.begin(), max_element(information_gains.begin(), information_gains.end()));
}
