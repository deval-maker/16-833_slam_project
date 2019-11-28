#include <controller.h>

Controller::Controller()
{
    goal = state_vector();
    error = state_vector();
    current_state = state_vector();
    prev_state = current_state;

    last_ts = std::chrono::high_resolution_clock::now();

    // cout << "[Controller] Drive type: " << drive_type << endl;
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

    // cout << "[Set Goal] " << goal.toStr() << endl;

}

void Controller::set_current_state(double x, double y, double theta)
{
    current_state.x = x;
    current_state.y = y;
    current_state.theta = theta;

    prev_state = current_state;

    // cout << "[Set current state] " << current_state.toStr() << endl;
}

void Controller::set_current_state(node n1)
{
    current_state.x = (double) n1.x;
    current_state.y = (double) n1.y;
    current_state.theta = (double) n1.theta;

    prev_state = current_state;

    // cout << "[Set current state] " << current_state.toStr() << endl;
}

void Controller::set_node_state(node &n1)
{
    n1.x = (int)current_state.x;
    n1.y = (int)current_state.y;
    n1.theta = current_state.theta;
}

bool Controller::is_goal_reached()
{
    if(fabs(error.x) <= goal_thresh.x && fabs(error.y) <= goal_thresh.y /*&& fabs(error.theta) <= goal_thresh.theta*/)// 0.9 for x and y, 2 degrees for theta
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Controller::threshold_max_control_sig()
{
    // Set maximum
    if(fabs(control_sig.linx) >= vel_max_thresh.linx)
    {
        if(control_sig.linx >= 0)
        {
            control_sig.linx = vel_max_thresh.linx;
        }
        else
        {
            control_sig.linx = -vel_max_thresh.linx;
        }
    }

    if(fabs(control_sig.liny) >= vel_max_thresh.liny)
    {
        if(control_sig.liny >= 0)
        {
            control_sig.liny = vel_max_thresh.liny;
        }
        else
        {
            control_sig.liny = -vel_max_thresh.liny;
        }
    }

    if(fabs(control_sig.ang) >= vel_max_thresh.ang)
    {
        if(control_sig.ang >= 0)
        {
            control_sig.ang = vel_max_thresh.ang;
        }
        else
        {
            control_sig.ang = -vel_max_thresh.ang;
        }
    }
}

void Controller::threshold_min_control_sig()
{
    // Set minimum
    if(fabs(control_sig.linx) <= vel_min_thresh.linx)
    {
        if(control_sig.linx >= 0.0)
        {
            control_sig.linx = vel_min_thresh.linx;
        }
        else
        {
            control_sig.linx = -vel_min_thresh.linx;
        }
    }

    if(fabs(control_sig.liny) <= vel_min_thresh.liny)
    {
        if(control_sig.liny >= 0)
        {
            control_sig.liny = vel_min_thresh.liny;
        }
        else
        {
            control_sig.liny = -vel_min_thresh.liny;
        }
    }

    if(fabs(control_sig.ang) <= vel_min_thresh.ang)
    {
        if(control_sig.ang >= 0)
        {
            control_sig.ang = vel_min_thresh.ang;
        }
        else
        {
            control_sig.ang = -vel_min_thresh.ang;
        }
    }
}
double wrapMax_controller(double x, double max)
{
    return fmod(max + fmod(x, max), max);
}
/* wrap x -> [min,max) */
double wrap2pi_controller(double x)
{
    return -M_PI + wrapMax_controller(x + (M_PI), 2*M_PI );
}

bool Controller::next_time_step(velocities &control_sig_)
{
    error.x = goal.x - current_state.x;
    error.y = goal.y - current_state.y;
    error.theta = goal.theta - current_state.theta;
    // cout<<"[Current Pos Error] "<<"x: "<<error.x<<" y: "<<error.y<<" Theta: "<<error.theta<<endl;


    // cout << "[Error] " << error.toStr() << endl;
    // cout << "[Current state] " << current_state.toStr() << endl;
    // cout << "[Prev state] " << prev_state.toStr() << endl;

    auto now_ts = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration<double>(now_ts - last_ts).count();

    bool is_at_goal = is_goal_reached();

    if(!is_at_goal)
    {

        if(Steer == drive_type)
        {
            // Vel desired
            desired_vel.linx = pow(pow(error.x,2) + pow(error.y,2), 1.0/2);
            desired_vel.ang = wrap2pi_controller(atan2(goal.y - current_state.y, goal.x - current_state.x));
            // cout << "[Desired Velo] " << desired_vel.toStr() << endl;


            // Get current velocities
            current_vel.linx = (current_state.x - prev_state.x);
            current_vel.ang = current_state.theta;
            // cout << "[Current Velo] " << current_vel.toStr() << endl;



            // Control Signal
            control_sig.linx = (desired_vel.linx) * Kp_st_lx;

            control_sig.ang = (desired_vel.ang - current_vel.ang) * Kp_st_a;
            // cout << "[Control Signal] " << control_sig.toStr() << endl;

        }
        else if(Omni == drive_type)
        {
            control_sig.linx = (error.x / dt) * Kp_lx;
            control_sig.liny = (error.y / dt) * Kp_ly;
            control_sig.ang = (error.theta / dt) * Kp_a;
            // cout << "[Control Signal] " << control_sig.toStr() << endl;
        }
        else
        {
            cout << "[Critical] Invalid drive type !" <<  endl;
        }

        threshold_max_control_sig();


    }
    else
    {
        control_sig.linx = 0.0;
        control_sig.ang = 0.0;
        // cout << "[Controller] Goal reached." << endl;
    }

    prev_state = current_state;

    // // Motion Model
    if(Steer == drive_type)
    {
        dt = 0.1;
        current_state.x += ( control_sig.linx * dt * cos(current_state.theta));
        current_state.y += ( control_sig.linx * dt * sin(current_state.theta));
        current_state.theta += ( control_sig.ang * dt);
    }
    else if(Omni == drive_type)
    {
        current_state.x += ( control_sig.linx * dt );
        current_state.y += ( control_sig.liny * dt );
        current_state.theta += ( control_sig.ang * dt );
    }
    else
    {
        cout << "[Critical] Invalid drive type !" << endl;
    }

    last_ts = now_ts;

    control_sig_ = control_sig;

    return is_at_goal;
}
