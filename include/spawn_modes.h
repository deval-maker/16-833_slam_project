#pragma once

double random_limits(double min, double max)
{
  return min + (rand() / ( RAND_MAX / (max-min) ) );
}

Eigen::Vector3f sample_valid_point(MapReader* _map)
{
  Eigen::Vector3f sample;
  uint8_t value = 0;
  vector<double> disc_angles;

  for(double i = 0; i< 2*M_PI; i += 2*M_PI/4){

    disc_angles.push_back(i);

  }

  while(value!=255)
  {
    sample(0) = random_limits(0,MAP_SIZE_X);
    sample(1) = random_limits(0,MAP_SIZE_Y);
    // sample(2) = random_limits(0,2*M_PI);
    sample[2] = disc_angles[rand()%disc_angles.size()];
    value = _map->query_map(sample[1], sample[0]);
  }
  return sample;
}

vector<mode> spawn_modes(mode original_mode, Eigen::Matrix3f sigma_init,
  int final_modes_num, MapReader* _map)
{
  
  int initial_modes_num = 20000; 
  double weight_init = 1.0;
  double weight_threshold = 0.50;

  vector<mode> samples;

  for(int i = 0; i < initial_modes_num - 1; i++)
  {
    mode sample_mode(sample_valid_point(_map), sigma_init, weight_init);
    samples.push_back(sample_mode);
    samples[i].visualize_ellipse(_map);
  }

  Eigen::Vector3f sample_mean(150,800,-M_PI/2);
  mode sample_mode(sample_mean, sigma_init, weight_init);
  samples.push_back(sample_mode);
  // Eigen::Vector3f sample_mean2(150,800,3.14/2);
  // mode sample_mode2(sample_mean2, sigma_init, weight_init);
  // samples.push_back(sample_mode2);

  _map->viz_session();
  _map->clear_session();

  vector<meas> original_measurements = _map->get_landmark_measurement(original_mode.mean);
  

  while(samples.size() > final_modes_num)
  {
    _map->clear_session();
    for(int i = 0; i < samples.size(); i++)
    {

      samples[i].update_weight(original_measurements,_map);
      // std::cout<<"I "<<i<<" Weight "<<samples[i].weight<<'\n';
      // if(i == samples.size() -1 || i == samples.size()-2 || i == samples.size()-3){
      //   std::cout<<samples[i].toStr()<<"\n";

      // }
      if(samples[i].weight < weight_threshold)
      {
        samples.erase(samples.begin() + i);
        // std::cout<<"Size of samples "<<samples.size()<<'\n';
        if(samples.size() == final_modes_num) break;
        i--;
      }
      else samples[i].visualize_ellipse(_map);
    }
    // std::cout<<"\n\n";
    // _map->viz_session();
  }
  return samples;
  // for()

}