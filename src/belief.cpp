#include<belief.h>
#include<MapReader.h>

mode::mode(){
    R << 0.001, 0, 0,
         0, 0.001, 0,
         0, 0, 0.001;
    Q << 0.001, 0,
         0, 0.001; 
    delT = 0.1;
}
mode::mode(Eigen::Vector3f mean, Eigen::Matrix3f sigma,int weight)
{
    R << 0.001, 0, 0,
        0, 0.001, 0,
        0, 0, 0.001;

    Q << 0.001, 0,
         0, 0.001;
    this->mean = mean;
    this->sigma = sigma;
    this->weight = weight;
    delT = 0.1;
}

Eigen::Matrix3f mode::getGt(double v, double theta)
{
    Eigen::Matrix3f matrix;
    matrix<<1, 0, -v*delT*sin(theta),
            0, 1, v*delT*cos(theta),
            0, 0, 1;
    return matrix;
}

Eigen::MatrixXf mode::getHt(double q, Eigen::Vector2f delta)
{
    Eigen::MatrixXf matrix(2,3);
    matrix<<delta[0]/sqrt(q), -delta[1]/sqrt(q), 0,
            delta[0]/q,          delta[1]/q,        -1;
    
    return matrix;
}

double wrapMax(double x, double max)
{
    return fmod(max + fmod(x, max), max);
}
/* wrap x -> [min,max) */
double wrap2pi(double x)
{
    return -M_PI + wrapMax(x + (M_PI), 2*M_PI );
}

void mode::propagate_mode(double v, double omega,vector<meas> &gt_meas,MapReader* map ){
    Eigen::Vector3f mean_bar;

    mean_bar[0] = mean[0] + v*delT*cos(mean[2]);
    mean_bar[1] = mean[1] + v*delT*sin(mean[2]);
    mean_bar[2] = mean[2] + omega*delT;

    Eigen::Matrix3f Gt = getGt(v,mean[2]);

    Eigen::Matrix3f sigma_bar = ((Gt * sigma) * Gt.transpose()) + R; 
    
    mean = mean_bar;
    sigma = sigma_bar;
    // mean[0] = mean_bar[0];
    for(int i = 0; i < gt_meas.size(); i++)
    {
        // map->getmeas(gt_meas.id, mean_bar, )
        point_t lm_position = map->get_landmark_pose(gt_meas[i].landmark_id);
        Eigen::Vector2f delta;
        delta[0] = lm_position[0] - mean_bar[0];
        delta[1] = lm_position[1] - mean_bar[1];

        double q = delta.transpose() * delta;

        meas measurement = map->get_measurement(gt_meas[i].landmark_id, mean_bar);
        Eigen::Vector2f actual_z(measurement.dist, measurement.psi); 

        Eigen::Vector2f predicted_z;
        predicted_z[0] = sqrt(q);
        predicted_z[1] = wrap2pi(atan2(delta[1],delta[0]) - mean_bar[2]);

        Eigen::MatrixXf Ht = getHt(q, delta);

        Eigen::MatrixXf Kt(3,2);
        Kt = sigma_bar * Ht.transpose() * (Ht * sigma_bar * Ht.transpose() + Q).inverse();

        mean = mean.eval() + Kt * (actual_z - predicted_z);  
        sigma = sigma.eval() - Kt * Ht * sigma_bar;
    }   
}


void mode::update_weight(vector<meas> &gt_meas, MapReader* map)
{
    double distance = 0;
    for(int i = 0; i < gt_meas.size(); i++)
    {
        meas observed_measurement = map->get_measurement(gt_meas[i].landmark_id, mean);

        Eigen::MatrixXf actual_z(2,1) ;
        actual_z << gt_meas[i].dist, gt_meas[i].psi;
        Eigen::MatrixXf measured_z(2,1);
        measured_z <<observed_measurement.dist, observed_measurement.psi;

        distance += ((actual_z - measured_z).transpose() * R.inverse() * (actual_z - measured_z)).value();  
    }

    weight *= exp(-0.5*distance);
}