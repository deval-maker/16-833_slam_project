#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <random>
#include <math.h>
#include <functional>
#include <node.h>


node::node(int id, int x, int y, double theta)
{
    this->id = id;
    this->x = x;
    this->y = y;
    this->theta = theta;
    
}

int node::similarity(node &node1, node &node2)
{
    int sim_count = 0;
    if(node1.visible_landmarks.size() < node2.visible_landmarks.size()){
        for(int landmark_id: node1.visible_landmarks){
            if(node2.visible_landmarks.find(landmark_id) != node2.visible_landmarks.end()){
                sim_count++;
            }
        }
    }

    else if(node1.visible_landmarks.size() >= node2.visible_landmarks.size()){
        for(int landmark_id: node2.visible_landmarks){
            if(node1.visible_landmarks.find(landmark_id) != node1.visible_landmarks.end()){
                sim_count++;
            }
        }
    }
    return sim_count;
}