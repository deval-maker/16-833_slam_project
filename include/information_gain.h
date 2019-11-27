point_t get_control(point_t start_state, point_t goal_state)
{
    return point_t();
}


void get_optimal_policy(vector<vector<point_t>> paths, vector<mode> modes,
MapReader* _map)
{
    double wt_threshold = 0.2;

    for(int i = 0; i < paths.size(); i++)
    {
        vector<point_t> path = paths[i];
        int information_gain = 0;
        for(int j = 0; j < modes.size(); j++)
        {
            vector<mode> modes_copy = modes;

            for(int t = 0; t < path.size(); t++)
            {
                double goal[2];

                point_t start_state, goal_state; 
                start_state.push_back(modes_copy[j].mean[0]);
                start_state.push_back(modes_copy[j].mean[1]);

                goal_state.push_back(path[t][0] + modes_copy[j].mean[0]);
                goal_state.push_back(path[t][1] + modes_copy[j].mean[1]);

                point_t control = get_control(start_state, goal_state);
                modes_copy[j].propagate_motion(control[0], control[1]);

                Eigen::Vector3f pose_mode_j = modes_copy[j].mean; 
                vector<meas> actual_meas = _map->get_landmark_measurement(pose_mode_j);

                for(int k = 0; k < modes_copy.size(); k++)
                {
                    if(k == j) continue;

                    point_t start_state, goal_state; 
                    start_state.push_back(modes_copy[k].mean[0]);
                    start_state.push_back(modes_copy[k].mean[1]);

                    goal_state.push_back(path[t][0] + modes_copy[k].mean[0]);
                    goal_state.push_back(path[t][1] + modes_copy[k].mean[1]);

                    point_t control = get_control(start_state, goal_state);

                    modes_copy[k].propagate_mode(control[0], control[1], actual_meas, _map);
                    modes_copy[k].update_weight(actual_meas, _map);
                }
            }

            for(int k = 0; k < modes_copy.size(); k++)
            {
                if(modes_copy[k].weight < wt_threshold)
                    information_gain++;
            }
        }
    }
}


