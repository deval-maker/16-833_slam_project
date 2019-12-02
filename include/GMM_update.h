#pragma once

vector<point_t> get_control_propagate(point_t start_state, point_t goal_state, MapReader* map_obj)
{

    // std::cout<<"start state received "<<start_state[0]<<" "<<start_state[1]<<" "<<start_state[2]<<'\n';
    // std::cout<<"Goal state received "<<goal_state[0]<<" "<<goal_state[1]<<"\n";
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
        step_vels.push_back(steer_velos.linx);
        step_vels.push_back(steer_velos.ang);
        controls.push_back(step_vels);

        // map_obj->update_visible_landmarks(n1, true);


    }


    return controls;
}


// vector<point_t> convert_to_path_propagate(point_t start_state, vector<point_t> actions)
// {
//     // std::cout<<"Start state received "<<start_state[0]<<" "<<start_state[1]<<'\n';  
//     vector<point_t> plan;
//     plan.push_back(start_state);
//     point_t current = start_state;
//     for(int i = actions.size() - 1; i >= 0; i--)
//     {
//         current[0] += actions[i][0];
//         current[1] += actions[i][1];
//         // std::cout<<"Actions "<<actions[i][0]<<' '<<actions[i][1]<<'\n';
//         plan.push_back(current);
//     }
//     return plan;
// }


vector<point_t> convert_to_path_propagate(point_t start_state, vector<point_t> actions, double relative_angle)
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

void propagate_policy(vector<point_t> actions, mode groundTruth, vector<mode> modes, 
MapReader* _map, int index_optimal_policy)
{
    std::cout<<"[INFO] Propogating modes using Optimal Policy"<<"\n";
    point_t groundTruth_start_state{groundTruth.mean[0], groundTruth.mean[1]};
    double wt_threshold = pow(10,-5);


    _map->clear_session();

    for(int i = 0; i < modes.size(); i++)
    {
        std::cout<<"Final mode "<<modes[i].mean[0]<<" "<<modes[i].mean[1]<<" "<<modes[i].mean[2]<<'\n';
    }

    std::cout<<"Ground truth start state "<<groundTruth_start_state[0]<<' '<<
    
    groundTruth_start_state[1]<<" "<< groundTruth.mean[2]<<"\n\n";

    double relative_angle = modes[index_optimal_policy].mean[2] - groundTruth.mean[2]; 

    vector<point_t> groundTruth_plan = convert_to_path_propagate(groundTruth_start_state, actions, relative_angle);

    std::vector<std::vector<point_t>> plans;

    for(int m = 0; m < modes.size(); m++)
    {
        point_t start{modes[m].mean[0], modes[m].mean[1]};
        double relative_angle = modes[index_optimal_policy].mean[2] - modes[m].mean[2]; 
        plans.push_back(convert_to_path_propagate(start, actions, relative_angle));
    }
 
    for(int t = 0; t < plans[0].size(); t++)
    {
        _map->clear_session();

        point_t start_state, goal_state;
        start_state.push_back(groundTruth.mean[0]);
        start_state.push_back(groundTruth.mean[1]);
        start_state.push_back(groundTruth.mean[2]);

        goal_state.push_back(groundTruth_plan[t][0]);
        goal_state.push_back(groundTruth_plan[t][1]);

        vector<point_t> controls = get_control_propagate(start_state, goal_state, _map);

        // std::cout<<"Ground Truth before propagation "<<groundTruth.mean[0]<<' '<<
        // groundTruth.mean[1]<<" "<<groundTruth.mean[2]<<'\n';

        for(auto control: controls){
            groundTruth.propagate_motion(control[0], control[1]);
        }
        point_t gt_current_state{groundTruth.mean[0], groundTruth.mean[1], groundTruth.mean[2]};

        _map->visualize_point_and_dir(gt_current_state, cv::viz::Color::green());

        // std::cout<<"Ground current state "<<groundTruth.mean[0]<<' '<<
        // groundTruth.mean[1]<<" "<<groundTruth.mean[2]<<'\n';

        
        vector<meas> actual_meas = _map->get_landmark_measurement(groundTruth.mean);


        for(int k = 0; k < modes.size(); k++)
        {
            point_t start_state, goal_state;

            start_state.push_back(modes[k].mean[0]);
            start_state.push_back(modes[k].mean[1]);
            start_state.push_back(modes[k].mean[2]);

            goal_state.push_back(plans[k][t][0]);
            goal_state.push_back(plans[k][t][1]);

            vector<point_t> controls = get_control_propagate(start_state, goal_state, _map);

            // std::cout<<"Controls state for mode "<<k<<" Before propagation "<<modes[k].mean[0]<<' '<<
            // modes[k].mean[1]<<" "<<modes[k].mean[2]<<'\n';



            for(auto control: controls){
                modes[k].propagate_motion(control[0], control[1]);
        
            }

            // std::cout<<"Controls state for mode "<<k<<" "<<modes[k].mean[0]<<' '<<
            // modes[k].mean[1]<<" "<<modes[k].mean[2]<<'\n';

            // modes[k].visualize_ellipse(_map);
            // modes_copy[k].update_measurement(actual_meas,_map);
            modes[k].update_weight(actual_meas, _map,t);

            point_t current_state{modes[k].mean[0], modes[k].mean[1], modes[k].mean[2]};

            if(modes[k].weight > wt_threshold)
                _map->visualize_point_and_dir(current_state, cv::viz::Color::red());

        }

        for(int i = 0; i < modes.size(); i++)
        {
            double weight_sum = 0;
            for(int i = 0; i < modes.size(); i++)
                weight_sum += modes[i].weight; 
            
            modes[i].weight /= weight_sum;

            std::cout<<"Intermediate weight of mode : "<<i<<" "<<modes[i].weight<<'\n';
        }

        std::cout<<"\n\n";
        _map->viz_session();
    }
    
    for(int i = 0; i < modes.size(); i++)
    {
        std::cout<<"Final weight "<<modes[i].weight<<'\n';
    }


}
