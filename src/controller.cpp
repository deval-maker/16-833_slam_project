#include <controller.h>

Controller::Controller()
{
    goal = state_vector();
    error = state_vector();
    current_state = state_vector(); 
    prev_state = current_state;

    last_ts = std::chrono::high_resolution_clock::now();
}

Controller::~Controller()
{

}

void Controller::set_goal(double x, double y, double theta)
{
    goal.x = x;
    goal.y = y;
    goal.theta = theta;

    last_ts = std::chrono::high_resolution_clock::now();

    cout << "[Set Goal] " << goal.toStr() << endl;

}

void Controller::set_current_state(double x, double y, double theta)
{
    current_state.x = x;
    current_state.y = y;
    current_state.theta = theta;

    prev_state = current_state;

    cout << "[Set current state] " << current_state.toStr() << endl;
}

bool Controller::is_goal_reached()
{
    if(fabs(error.x) <= goal_thresh.x && fabs(error.y) <= goal_thresh.y && fabs(error.theta) <= goal_thresh.theta)// 0.9 for x and y, 2 degrees for theta
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Controller::next_time_step()
{
    error.x = goal.x - current_state.x;
    error.y = goal.y - current_state.y;
    error.theta = goal.theta - current_state.theta;

    cout << "[Error] " << error.toStr() << endl;
    cout << "[Current state] " << current_state.toStr() << endl;
    cout << "[Prev state] " << prev_state.toStr() << endl;

    auto now_ts = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration<double>(now_ts - last_ts).count();
    
    bool is_at_goal = is_goal_reached();

    if(!is_at_goal)
    {

        // Vel desired
        desired_vel.lin = sqrt( pow((error.x / dt),2)  + pow((error.y / dt),2) );
        desired_vel.ang = error.theta / dt;

        cout << "[Desired Velo] " << desired_vel.toStr() << endl;


        // Get current velocities
        current_vel.lin = sqrt( pow(( (current_state.x - prev_state.x) / dt),2)  + pow(((current_state.y - prev_state.y) / dt),2) );
        current_vel.ang = (current_state.theta - prev_state.theta) / dt;

        cout << "[Current Velo] " << current_vel.toStr() << endl;


        // Control Signal
        control_sig.lin = (desired_vel.lin - current_vel.lin) * Kp_l;
        control_sig.ang = (desired_vel.ang - current_vel.ang) * Kp_o;

        cout << "[Control Signal] " << control_sig.toStr() << endl;

        // Set minimum
        if(fabs(control_sig.lin) <= vel_min_thresh.lin)
        {
            control_sig.lin = vel_min_thresh.lin;
        }
        
        if(fabs(control_sig.ang) >= vel_min_thresh.ang)
        {
            control_sig.ang = vel_min_thresh.ang;
        }

        // Set maximum
        if(fabs(control_sig.lin) >= vel_max_thresh.lin)
        {
            control_sig.lin = vel_max_thresh.lin;
        }
        
        if(fabs(control_sig.ang) >= vel_max_thresh.ang)
        {
            control_sig.ang = vel_max_thresh.ang;
        }
    }
    else
    {
        control_sig.lin = 0.0;
        control_sig.ang = 0.0;
        cout << "[Controller] Goal reached." << endl;
    }

    prev_state = current_state;

    // Motion Model 
    current_state.x += ( control_sig.lin * dt * cos(current_state.theta));
    current_state.y += ( control_sig.lin * dt * sin(current_state.theta));
    current_state.theta += ( control_sig.ang * dt);

    // Add Noise
    // TODO:

    last_ts = now_ts;

    return is_at_goal;
}