#include <utils.h>
#include <controller.h>

point_t get_control(point_t start_state, point_t goal_state)
{
       point_t ret; 
    velocities omni_velos;

    Controller c1 = Controller();
    c1.set_current_state(400.0, 600.0, 0.0);
    c1.set_goal(406.0, 606.0, PI/2);

    c1.next_time_step(omni_velos);

    ret.push_back(omni_velos.linx);
    ret.push_back(omni_velos.liny);
    ret.push_back(omni_velos.ang);

    return ret;
}

int get_optimal_policy(vector<vector<point_t>> paths, vector<mode> modes,
MapReader* _map)
{
    double wt_threshold = 0.2;
    std::vector<double> information_gains;

    for(int i = 0; i < paths.size(); i++)
    {
        vector<point_t> path = paths[i];
        int information_gain_policy = 0;

        for(int j = 0; j < modes.size(); j++)
        {
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
                    information_gain_mode++;
            }

            information_gain_policy += modes[j].weight * information_gain_mode;

        }
        information_gains.push_back(information_gain_policy);
    }
    return distance(information_gains.begin(), max_element(information_gains.begin(), information_gains.end()));
}


