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
        Eigen::Matrix3f R;
        Eigen::Matrix2f Q;
        void propagate_mode(double v, double omega,vector<meas> &gt_meas,MapReader* map );
        void propagate_motion(double v, double omega);
        void update_weight(vector<meas> &gt_meas,MapReader* map);
        mode();
        mode(Eigen::Vector3f mean, Eigen::Matrix3f sigma,int weight);
        Eigen::Matrix3f getGt(double v, double theta);
        Eigen::MatrixXf getHt(double q, Eigen::Vector2f delta);
    private:





    
};