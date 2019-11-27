#pragma once

#include <utils.h>

class node
{
    public:
    int id;
    int x;
    int y;
    double theta;
    std::unordered_set<int> visible_landmarks;

    node()
    {

    }

    node(int id, int x, int y, double theta);
    friend ostream& operator<<(ostream& os, const node& dt);
    static int similarity(node &node1, node &node2);
    
};


class meas
{
    public:
    int landmark_id;
    double dist;
    double psi;
    bool is_visible;

    meas(){

    }
    meas(int landmark_id, double dist, double psi){
        this->landmark_id = landmark_id;
        this->dist = dist;
        this->psi = psi;
    }

    string toStr()
    {
        return  "Landmark ID: " + to_string(landmark_id) + " Distance: " + to_string(dist) +  " Bearing Angle: " +  to_string((psi*180)/PI) + " degrees";
    }
    friend bool operator== (const meas &m1, const meas &m2);
};

bool operator== (const meas &m1, const meas &m2)
{
    return m1.landmark_id == m2.landmark_id;
}