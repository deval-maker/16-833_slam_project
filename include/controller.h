#pragma once

#include <utils.h>

class velocities
{
public:
    double linx;
    double liny;
    double ang;

    velocities()
    {
        linx = 0.0;
        liny = 0.0;
        ang = 0.0;
    }

    velocities(double _linx, double _liny, double _ang)
    {
        linx = _linx;
        liny = _liny;
        ang = _ang;
    }

    string toStr()
    {
        return  "linear velo X: " + to_string(linx) + " linear velo Y: " + to_string(liny) +  " angular velo: " +  to_string(ang);
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

    double Kp_lx = 1;
    double Kp_ly = 1;
    double Kp_a = 0.8;

    double Kd_lx = 0;
    double Kd_ly = 0;
    double Kd_a = 0;

    bool is_goal_reached();
    void threshold_max_control_sig();
    void threshold_min_control_sig();
    
public:
    
    enum drive_type_e
    {
        Steer, 
        Omni
    };

    state_vector goal_thresh = state_vector(0.5, 0.5, (2*PI)/180.0);
    velocities vel_max_thresh = velocities(2.0, 2.0, 1.5);
    velocities vel_min_thresh = velocities(0.1, 0.1, 0.01);

    state_vector current_state; 
    drive_type_e drive_type = Omni;

    Controller();
    ~Controller();

    void set_goal(double x, double y, double theta);
    void set_current_state(double x, double y, double theta);
    bool next_time_step();

};