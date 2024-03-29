#include<belief.h>
#include<MapReader.h>

mode::mode(){
    R << 0.00, 0, 0,
         0, 0.00, 0,
         0, 0, 0.00;

   GMM_R << 1000, 0,
            0, 0.3;

    Q << 0.00, 0,
         0, 0.00;

    beta = 0;
    delT = 0.1;
}
mode::mode(Eigen::Vector3f mean, Eigen::Matrix3f sigma, double weight)
{
    R << 0.00, 0, 0,
        0, 0.00, 0,
        0, 0, 0.00;

    GMM_R << 10000, 0,
             0, 0.3;

    Q << 0.00, 0,
         0, 0.00;
    
    beta = 0;
    
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
    // matrix<<delta[1]/q,          -delta[0]/q,        -1,
    //         -delta[0]/sqrt(q),   -delta[1]/sqrt(q),   0;
    matrix<<-delta[0]/sqrt(q),          -delta[1]/sqrt(q),        0,
             delta[1]/sqrt(q),          -delta[0]/sqrt(q),       -1;

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
    vector<meas> measurements = map->get_landmark_measurement(mean_bar);

    for(int i = 0; i < gt_meas.size(); i++)
    {
        // map->getmeas(gt_meas.id, mean_bar, )
        meas measurement;
        auto it = std::find(measurements.begin(), measurements.end(), gt_meas[i]);
        if(it == measurements.end()) continue;
        else measurement = *it;

        point_t lm_position = map->get_landmark_pose(gt_meas[i].landmark_id);
        Eigen::Vector2f delta;
        delta[0] = lm_position[0] - mean_bar[0];
        delta[1] = lm_position[1] - mean_bar[1];

        double q = delta.transpose() * delta;



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

void mode::propagate_motion(double v, double omega)
{
    // std::cout<<"Before state "<<mean[0]<<" "<<mean[1]<<" "<<mean[2]<<'\n';
    // std::cout<<"Velocity "<<v<<" Omega "<<omega<<" Time "<<delT<<'\n';
    // std::cout<<"delx "<<v*delT*cos(mean[2])<<" dely "<<v*delT*sin(mean[2])<<
    // "delTheta"<<omega*delT<<'\n';
    Eigen::Vector3f mean_bar;
    mean_bar[0] = mean[0] + v*delT*cos(mean[2]);
    mean_bar[1] = mean[1] + v*delT*sin(mean[2]);
    mean_bar[2] = wrap2pi(mean[2] + omega*delT);


    Eigen::Matrix3f Gt = getGt(v,mean[2]);

    Eigen::Matrix3f sigma_bar = ((Gt * sigma) * Gt.transpose()) + R;

    mean = mean_bar;
    sigma = sigma_bar;
    // std::cout<<"Current state "<<mean[0]<<" "<<mean[1]<<" "<<mean[2]<<'\n';

}


void mode::update_measurement(vector<meas> &gt_meas, MapReader* map)
{
    vector<meas> measurements = map->get_landmark_measurement(mean);

    for(int i = 0; i < gt_meas.size(); i++)
    {
        // map->getmeas(gt_meas.id, mean_bar, )
        meas measurement;
        auto it = std::find(measurements.begin(), measurements.end(), gt_meas[i]);
        if(it == measurements.end()) continue;
        else measurement = *it;


        //std::cout<<"Gt measurement "<<gt_meas[i].landmark_id<<" "<<gt_meas[i].dist<<" "<<
        //gt_meas[i].psi<<'\n';

        //std::cout<<"Observed measurements "<<measurement.landmark_id<<" "<<measurement.dist<<" "<<
        //measurement.psi<<'\n';

        point_t lm_position = map->get_landmark_pose(gt_meas[i].landmark_id);
        Eigen::Vector2f delta;

        std::cout<<"Actual landmark pose "<<lm_position[0]<<" "<<lm_position[1]<<'\n';
        delta[0] = lm_position[0] - mean[0]; //dx
        delta[1] = lm_position[1] - mean[1]; //dy

        double q = delta.transpose() * delta;

        std::cout<<"Q value "<<q<<"\n";

        Eigen::Vector2f actual_z(gt_meas[i].dist, gt_meas[i].psi);
        Eigen::Vector2f observed_z(measurement.dist, measurement.psi);

        Eigen::Vector2f predicted_z;
        predicted_z[0] = sqrt(q);
        predicted_z[1] = wrap2pi(atan2(delta[1],delta[0]) - mean[2]);



        Eigen::MatrixXf Ht = getHt(q, delta);


        Eigen::MatrixXf Kt(3,2);
        Kt = sigma * Ht.transpose() * (Ht * sigma * Ht.transpose() + Q).inverse();

        std::cout<<"Current Mode Mean "<<mean[0]<<" "<<mean[1]<<" "<<mean[2]*180/M_PI<<"\n";
        std::cout<<"Actual Z "<<actual_z[0]*180/M_PI<<" "<<actual_z[1]<<'\n';
        std::cout<<"Observed Z "<<observed_z[0]*180/M_PI<<" "<<observed_z[1]<<'\n';
        std::cout<<"Predicted Z "<<predicted_z[0]*180/M_PI<<" "<<predicted_z[1]<<'\n';

        mean = mean.eval() + Kt * (observed_z - actual_z);
        mean[2] = wrap2pi(mean[2]);
       
        sigma = sigma.eval() - Kt * Ht * sigma.eval();
        std::cout<<"Kalman Gain "<<"\n"<<Kt<<"\n";
        std::cout<<"Product "<<"\n"<<Kt*Ht<<"\n";
        std::cout<<"Measurement jacobian "<<"\n"<<Ht<<"\n";
        std::cout<<"Corrected Mean "<<mean[0]<<" "<<mean[1]<<" "<<mean[2]*180/M_PI<<"\n";
       
        std::cout<<'\n';
    }

}

void mode::update_weight(vector<meas> &gt_meas, MapReader* map, int time_step, double factor, 
double angle_factor, double dist_factor)
{
    double distance = 0;
    vector<meas> observed_measurements = map->get_landmark_measurement(mean);
    int common_landmarks = 0;
    // std::cout<<"\n\n Updating weight for new mode \n";
    for(int i = 0; i < gt_meas.size(); i++)
    {
        auto it = std::find(observed_measurements.begin(), observed_measurements.end(), gt_meas[i]);
        if(it == observed_measurements.end()) continue;
        // std::cout<<"Common Landmark \n";
        common_landmarks += 1;
        meas observed_measurement = *it;

        Eigen::MatrixXf actual_z(2,1) ;
        actual_z << gt_meas[i].dist, gt_meas[i].psi;

        Eigen::MatrixXf measured_z(2,1);
        measured_z << observed_measurement.dist, observed_measurement.psi;

        Eigen::MatrixXf difference_z(2,1);
        difference_z = actual_z - measured_z;
        // difference_z(1,0) = wrap2pi(difference_z(1,0));
        difference_z(1,0) = angle_factor * atan2(sin(gt_meas[i].psi-observed_measurement.psi), cos(gt_meas[i].psi-observed_measurement.psi));
        difference_z(0,0) = dist_factor * difference_z(0,0);


        // std::cout<<"Landmark ID "<<gt_meas[i].landmark_id<<" "<<observed_measurement.landmark_id<<'\n';
        // std::cout<<"Actual z \n"<<gt_meas[i].toStr()<<'\n';
        // std::cout<<"measured z \n"<<observed_measurement.toStr()<<'\n';
        // std::cout<<"Difference \n"<<difference_z<<'\n';
        // std::cout<<"Inverse \n"<<GMM_R.inverse()<<'\n';

        distance += (difference_z.transpose() * GMM_R.inverse() * difference_z).value();
    }
    // std::cout<<"Distance "<<distance<<"\n";
    // std::cout<<"Number of Common Landmarks "<<common_landmarks<<'\n';
    // if(common_landmarks > 0) std::cout<<"distance is "<<distance<<'\n';
    weight *= exp(-0.5*distance);

    if(common_landmarks != observed_measurements.size() || 
        common_landmarks != gt_meas.size())
    {
        double alpha = std::max(1 + observed_measurements.size() - common_landmarks, 
        1 + gt_meas.size() - common_landmarks);
        beta += delT;
        double gamma = exp(-factor*alpha*beta);
        weight *= gamma; 
        // std::cout<<"Gamma "<<gamma<<'\n';
    }   

}

void mode::visualize_ellipse(MapReader* map)
{
    map->visualize_ellipse(mean.block<2,1>(0,0), sigma.block<2,2>(0,0));
}
