#pragma once

#include <utils.h>

class velocities
{
public:
    double lin;
    double ang;

    velocities()
    {
        lin = 0.0;
        ang = 0.0;
    }

    velocities(double _lin, double _ang)
    {
        lin = _lin;
        ang = _ang;
    }

    string toStr()
    {
        return  "linear velo: " + to_string(lin) + " angular velo: " +  to_string(ang);
    }
};

class state_vector
{
public:
    double x;
    double y;
    double theta;

    state_vector()
    {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
    }

    state_vector(double _x, double _y, double _theta)
    {
        x = _x;
        y = _y;
        theta = _theta;
    }

    string toStr()
    {
        return  "x: " + to_string(x) + " y: " +  to_string(y) + " theta: " + to_string(theta);
    }
};

class Controller
{

private:
    state_vector goal;
    state_vector error;
    state_vector prev_state;

    std::chrono::high_resolution_clock::time_point last_ts;
    
    velocities control_sig;
    velocities desired_vel;
    velocities current_vel;

    double Kp_l = 1;
    double Kp_o = 10;

    bool is_goal_reached();

public:
    
    state_vector goal_thresh = state_vector(0.5, 0.5, (2*PI)/180.0);
    velocities vel_max_thresh = velocities(2, 10); // 2m/s and 1rad/s
    velocities vel_min_thresh = velocities(0.1, 0.01); // 2m/s and 1rad/s


    state_vector current_state; 
    
    Controller();
    ~Controller();

    void set_goal(double x, double y, double theta);
    void set_current_state(double x, double y, double theta);
    bool next_time_step();
};