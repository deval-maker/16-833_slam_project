#pragma once

#include<iostream>
#include<MapReader.h>
#include<eigen3/Eigen/Dense>
#include<node.h>



class mode{

    public:
        int id;
        Eigen::Vector3f mean;
        Eigen::Matrix3f sigma;
        double delT;
        double weight;
        double beta;
        Eigen::Matrix3f R;
        Eigen::Matrix2f GMM_R;
        Eigen::Matrix2f Q;
        void propagate_mode(double v, double omega,vector<meas> &gt_meas,MapReader* map );
        void propagate_motion(double v, double omega);
        void update_measurement(vector<meas> &gt_meas, MapReader* map);
        void update_weight(vector<meas> &gt_meas,MapReader* map);
        mode();
        mode(Eigen::Vector3f mean, Eigen::Matrix3f sigma,double weight);
        Eigen::Matrix3f getGt(double v, double theta);
        Eigen::MatrixXf getHt(double q, Eigen::Vector2f delta);
        void visualize_ellipse(MapReader* map);
     
        string toStr()
        {
            return  "x: " + to_string(mean[0]) + " y: " + to_string(mean[1]) +  " theta: " + to_string(mean[2]*180/M_PI) + " degrees" + " Weight: " + to_string(weight) ;
        }

    private:






};

