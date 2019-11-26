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

    meas(){

    }
    meas(int landmark_id, double dist, double psi){
        this->landmark_id = landmark_id;
        this->dist = dist;
        this->psi = psi;
    }
};