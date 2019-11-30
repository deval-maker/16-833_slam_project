#pragma once

double random_limits(double min, double max)
{
  return min + (rand() / ( RAND_MAX / (max-min) ) );
}

Eigen::Vector3f sample_valid_point(MapReader* _map)
{
  Eigen::Vector3f sample;
  uint8_t value = 0;
  while(value!=255)
  {
    sample(0) = random_limits(0,MAP_SIZE_X);
    sample(1) = random_limits(0,MAP_SIZE_Y);
    sample(2) = random_limits(0,2*M_PI);
    value = _map->query_map(sample[1], sample[0]);
  }
  return sample;
}

vector<mode> spawn_modes(mode original_mode, Eigen::Matrix3f sigma_init,
  int final_modes_num, MapReader* _map)
{
  
  int initial_modes_num = 50; 
  double weight_init = 1.0;
  double weight_threshold = 0.5;

  vector<mode> samples;

  for(int i = 0; i < initial_modes_num - 1; i++)
  {
    mode sample_mode(sample_valid_point(_map), sigma_init, weight_init);
    samples.push_back(sample_mode);
    samples[i].visualize_ellipse(_map);
  }
  Eigen::Vector3f sample_mean(147,640,0);
  mode sample_mode(sample_mean, sigma_init, weight_init);
  samples.push_back(sample_mode);

  _map->viz_session();
  _map->clear_session();

  vector<meas> original_measurements = _map->get_landmark_measurement(original_mode.mean);
  

  while(samples.size() > final_modes_num)
  {
    _map->clear_session();
    for(int i = 0; i < samples.size(); i++)
    {
      samples[i].update_weight(original_measurements,_map);
      std::cout<<"I "<<i<<" Weight "<<samples[i].weight<<'\n';

      if(samples[i].weight < weight_threshold)
      {
        samples.erase(samples.begin() + i);
        // std::cout<<"Size of samples "<<samples.size()<<'\n'; 
        i--;
      }
      else samples[i].visualize_ellipse(_map);
    }
    std::cout<<"\n\n";
    _map->viz_session();
  }
  return samples;
  // for()

}