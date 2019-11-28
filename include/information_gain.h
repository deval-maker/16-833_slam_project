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
        step_vels.push_back(steer_velos.linx);
        step_vels.push_back(steer_velos.ang);
        controls.push_back(step_vels);

        // map_obj->update_visible_landmarks(n1, true);


    }


    return controls;
}

int get_optimal_policy(vector<vector<point_t>> paths, vector<mode> modes,
MapReader* _map)
{
    std::cout<<"Inside get optimal policy \n";
    double wt_threshold = 0.2;
    std::vector<double> information_gains;

    for(int i = 0; i < paths.size(); i++)
    {
        std::cout<<"Path number "<<i<<'\n';
        vector<point_t> path = paths[i];
        int information_gain_policy = 0;

        for(int j = 0; j < modes.size(); j++)
        {
            std::cout<<"j th mode is "<<j<<'\n';
            int information_gain_mode = 0;

            vector<mode> modes_copy = modes;

            for(int t = 0; t < path.size(); t++)
            {
                double goal[2];

                point_t start_state, goal_state;
                start_state.push_back(modes_copy[j].mean[0]);
                start_state.push_back(modes_copy[j].mean[1]);

                goal_state.push_back(path[t][0] + modes_copy[j].mean[0]);
                goal_state.push_back(path[t][1] + modes_copy[j].mean[1]);

                vector<point_t> controls = get_control(start_state, goal_state, _map);

                std::cout<<"Start state "<<start_state[0]<<" "<<start_state[1]<<"\n";
                std::cout<<"Goal state "<<goal_state[0]<<" "<<goal_state[1]<<"\n";
                std::cout<<"Mode state "<<modes_copy[j].mean[0]<<" "<<modes_copy[j].mean[1]<<'\n';
                std::cout<<"Controls size "<<controls.size()<<'\n';
                for(auto control: controls){
                    modes_copy[j].propagate_motion(control[0], control[1]);

                }
                std::cout<<"Mode state after propagation "<<modes_copy[j].mean[0]<<" "<<modes_copy[j].mean[1]<<'\n';
                _map->viz_session();
                // std::cout<<"Mean of j: "<<modes_copy[j].mean[0]<<" "<<modes_copy[j].mean[1]<<" "<<
                // modes_copy[j].mean[2]<<'\n';

                modes_copy[j].visualize_ellipse(_map);

                // Eigen::Vector3f pose_mode_j = modes_copy[j].mean;
                // vector<meas> actual_meas = _map->get_landmark_measurement(pose_mode_j);
                //
                // for(int k = 0; k < modes_copy.size(); k++)
                // {
                //     if(k == j) continue;
                //
                //     point_t start_state, goal_state;
                //     start_state.push_back(modes_copy[k].mean[0]);
                //     start_state.push_back(modes_copy[k].mean[1]);
                //
                //     goal_state.push_back(path[t][0] + modes_copy[k].mean[0]);
                //     goal_state.push_back(path[t][1] + modes_copy[k].mean[1]);
                //
                //     vector<point_t> controls = get_control(start_state, goal_state, _map);
                //
                //     for(auto control: controls){
                //         modes_copy[k].propagate_motion(control[0], control[1]);
                //
                //     }
                //     modes_copy[k].update_measurement(actual_meas,_map);
                //     modes_copy[k].update_weight(actual_meas, _map);
                // }
            }
            _map->viz_session();

            for(int k = 0; k < modes_copy.size(); k++)
            {
                if(modes_copy[k].weight < wt_threshold)
                    information_gain_mode++;
            }
            information_gain_policy += modes[j].weight * information_gain_mode;

        }
        information_gains.push_back(information_gain_policy);
    }
    for(int i = 0; i<information_gains.size(); i++)
    {
    std::cout<<"Information gain "<<information_gains[i]<<'\n';

    }
    return distance(information_gains.begin(), max_element(information_gains.begin(), information_gains.end()));
}
