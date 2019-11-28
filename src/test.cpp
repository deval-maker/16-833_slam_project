#include <test.h>

void test_raycast1()
{
    String map_path = "data/map1.txt";
    MapReader map_obj = MapReader(map_path);

    // map_obj.visualize_map();

    cout << "[MapReader 1] Test Passed !" << endl;
}

void test_raycast2()
{
    String map_path = "data/map2.txt";
    MapReader map_obj = MapReader(map_path);

    // Make sure that the laser_fov is 360
    Eigen::Vector3f curr_pose = Eigen::Vector3f(400, 600, 0);
    vector<meas> land_meas;

    land_meas = map_obj.get_landmark_measurement(curr_pose);

    if(land_meas.size() == 2)
    {
        cout << "[MapReader 2] Test Passed !" << endl;
    }
    else
    {
        cout << "[MapReader 2] Test Failed !" << endl;
    }
}

void test_raycast3()
{
    String map_path = "data/map2.txt";
    MapReader map_obj = MapReader(map_path);

    point_t land_pose;
    int landmark_id = 1;

    land_pose = map_obj.get_landmark_pose(landmark_id);
    cout << "Landmark id: " << landmark_id << " X: " << land_pose[0] << " Y: " << land_pose[1] << endl;

    landmark_id = 2;
    land_pose = map_obj.get_landmark_pose(landmark_id);
    cout << "Landmark id: " << landmark_id << " X: " << land_pose[0] << " Y: " << land_pose[1] << endl;

    landmark_id = 3;
    land_pose = map_obj.get_landmark_pose(landmark_id);
    cout << "Landmark id: " << landmark_id << " X: " << land_pose[0] << " Y: " << land_pose[1] << endl;


    cout << "[MapReader 3] Test Passed !" << endl;
}

void test_controller1()
{
    // Motion in X direction
    Controller c1 = Controller();
    c1.set_current_state(400.0, 600.0, 0.0);
    c1.set_goal(406.0, 600.0, 0.0);

    bool is_goal_reached = false;

    auto start = std::chrono::high_resolution_clock::now();
    double time_elapsed = 0.0;

    while(!is_goal_reached)
    {
        // is_goal_reached = c1.next_time_step();

        // Sleep for 100ms
        std::chrono::duration<int, std::milli> sleep_time(100);
        std::this_thread::sleep_for(sleep_time);

        time_elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
        if(time_elapsed > 5.0)
        {
            break;
        }
    }

    if(is_goal_reached)
    {
        cout << "[Controller 1] Test Passed !" << " Total time: " << time_elapsed << endl;
    }
    else
    {
        cout << "[Controller 1] Test Failed !" << endl;
    }

    cout << "[Current state] " << c1.current_state.toStr() << endl;

}

void test_controller2()
{
    // Motoion in Y
    Controller c1 = Controller();
    c1.set_current_state(400.0, 600.0, 0.0);
    c1.set_goal(400.0, 606.0, 0.0);

    bool is_goal_reached = false;

    auto start = std::chrono::high_resolution_clock::now();
    double time_elapsed = 0.0;

    while(!is_goal_reached)
    {
        // is_goal_reached = c1.next_time_step();

        // Sleep for 100ms
        std::chrono::duration<int, std::milli> sleep_time(100);
        std::this_thread::sleep_for(sleep_time);

        time_elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
        if(time_elapsed > 5.0)
        {
            break;
        }
    }

    if(is_goal_reached)
    {
        cout << "[Controller 2] Test Passed !" << " Total time: " << time_elapsed << endl;
    }
    else
    {
        cout << "[Controller 2] Test Failed !" << endl;
    }

    cout << "[Current state] " << c1.current_state.toStr() << endl;

}

void test_controller3()
{
    // Motoion in Theta
    Controller c1 = Controller();
    c1.set_current_state(400.0, 600.0, 0.0);
    c1.set_goal(400.0, 600.0, PI/2);

    bool is_goal_reached = false;

    auto start = std::chrono::high_resolution_clock::now();
    double time_elapsed = 0.0;

    while(!is_goal_reached)
    {
        // is_goal_reached = c1.next_time_step();

        // Sleep for 100ms
        std::chrono::duration<int, std::milli> sleep_time(100);
        std::this_thread::sleep_for(sleep_time);

        time_elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
        if(time_elapsed > 5.0)
        {
            break;
        }
    }

    if(is_goal_reached)
    {
        cout << "[Controller 3] Test Passed !" << " Total time: " << time_elapsed << endl;
    }
    else
    {
        cout << "[Controller 3] Test Failed !" << endl;
    }

    cout << "[Current state] " << c1.current_state.toStr() << endl;
}

void test_controller4()
{
    // Motoion in X, Y, Theta
    Controller c1 = Controller();
    c1.set_current_state(400.0, 600.0, 0.0);
    c1.set_goal(406.0, 606.0, PI/2);

    bool is_goal_reached = false;

    auto start = std::chrono::high_resolution_clock::now();
    double time_elapsed = 0.0;

    while(!is_goal_reached)
    {
        // is_goal_reached = c1.next_time_step();

        // Sleep for 100ms
        std::chrono::duration<int, std::milli> sleep_time(100);
        std::this_thread::sleep_for(sleep_time);

        time_elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
        if(time_elapsed > 5.0)
        {
            break;
        }
    }

    if(is_goal_reached)
    {
        cout << "[Controller 4] Test Passed !" << " Total time: " << time_elapsed << endl;
    }
    else
    {
        cout << "[Controller 4] Test Failed !" << endl;
    }

    cout << "[Current state] " << c1.current_state.toStr() << endl;
}

void test_map_and_controller()
{
    // Motoion in X, Y, Theta in Map 2

    String map_path = "data/map2.txt";
    MapReader map_obj = MapReader(map_path);
    node n1 = node(1, 400, 600, 0);
    Controller c1 = Controller();

    c1.set_current_state(n1);
    c1.set_goal(600.0, 600.0, 0.0);

    bool is_goal_reached = false;
    auto start = std::chrono::high_resolution_clock::now();
    double time_elapsed = 0.0;

    map_obj.update_visible_landmarks(n1, true);
    int before =  n1.visible_landmarks.size();
    cout << "Visible Landmarks in the begining: " << before << endl;

    while(!is_goal_reached)
    {
        // is_goal_reached = c1.next_time_step();

        c1.set_node_state(n1);
        // cout << "x: " << n1.x  << " y: " << n1.y  << " theta: "<< n1.theta << endl;

        map_obj.update_visible_landmarks(n1, true);
        // cout << "Visible Landmarks: " << n1.visible_landmarks.size() << endl;

        // Sleep for 100ms
        std::chrono::duration<int, std::milli> sleep_time(100);
        std::this_thread::sleep_for(sleep_time);

        time_elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
        if(time_elapsed > 15.0)
        {
            break;
        }
    }

    int after =  n1.visible_landmarks.size();

    if(is_goal_reached && before != after)
    {
        cout << "[Controller 4] Test Passed !" << " Total time: " << time_elapsed << endl;
    }
    else
    {
        cout << "[Controller 4] Test Failed !" << endl;
    }

    cout << "Visible Landmarks after reaching: " << after << endl;
    cout << "[Current state] " << c1.current_state.toStr() << endl;
}

vector<point_t> convert_to_path_test(point_t start_state, vector<point_t> actions)
{
    vector<point_t> plan;
    plan.push_back(start_state);
    point_t current = start_state;
    for(int i = actions.size() - 1; i >= 0; i--)
    {
        current[0] += actions[i][0];
        current[1] += actions[i][1];
        // std::cout<<"Actions "<<actions[i][0]<<' '<<actions[i][1]<<'\n';
        plan.push_back(current);
    }
    return plan;
}

int closest_node(point_t point, std::vector<point_t> traj)
{
  int min_index = 0;
  double min_distance = INT_MAX;
  for(int i = 0; i < traj.size(); i++)
  {
    double distance = sqrt(pow(traj[i][0] - point[0], 2) + pow(traj[i][1] - point[1], 2));
    if(min_distance > distance)
    {
      min_distance = distance;
      min_index = i;
    }
  }
  return min_index;
}

    // point = np.array([X, Y])
    // traj = np.asarray(traj)
    // dist = point - traj
    // dist_2 = np.sum(dist ** 2, axis=1)
    // minIndex = np.argmin(dist_2)
    // return np.sqrt(dist_2[minIndex]), minIndex


void test_steer_controller()
{
    std::cout<<"Testing steer controller \n";
    String map_path = "data/map2.txt";
    MapReader map_obj = MapReader(map_path);
    map_obj.visualize_map();
// ----------------Planner------------------------------------
    Search wa_search;
    point_t start_point{100,400,1}, goal_point{800,820,0};
    wa_search.set_start(start_point);
    wa_search.set_goal(goal_point);
    wa_search.set_mapReader(&map_obj);
    vector<point_t> actions = wa_search.plan();
    vector<point_t> path = convert_to_path_test(start_point, actions);
    std::cout<<"Path size "<<path.size()<<'\n';
    map_obj.visualize_path(path, cv::viz::Color::green());
    map_obj.viz_session();

    reverse(actions.begin(), actions.end());

// ----------------Controller------------------------------------
    Controller c1 = Controller();
    node n1 = node(1, start_point[0], start_point[1], start_point[2]);
    c1.set_current_state(n1);
    c1.drive_type = Controller::Steer;

    auto start = std::chrono::high_resolution_clock::now();
    double time_elapsed = 0.0;
    velocities steer_velocities;

    map_obj.update_visible_landmarks(n1, true);

    goal_point = start_point;
    int lookAhead = 20;
    int nearestID;
    point_t current{n1.x,n1.y};
    bool is_goal_reached = false;
    for(int i = 0; i < path.size(); i++)
      std::cout<<path[i][0]<<" "<<path[i][1]<<'\n';
    while(!is_goal_reached || (nearestID + lookAhead) != path.size())
    {
      current[0] = n1.x;
      current[1] = n1.y;
      nearestID = closest_node(current, path);
      std::cout<<"nearestID "<<nearestID<<'\n';

      if(nearestID + lookAhead > path.size())
        lookAhead = path.size() - nearestID;

      goal_point[0] = path[nearestID + lookAhead][0];
      goal_point[1] = path[nearestID + lookAhead][1];
      c1.set_goal(goal_point[0], goal_point[1], goal_point[2]);

      is_goal_reached = c1.next_time_step(steer_velocities);
      c1.set_node_state(n1);
      map_obj.update_visible_landmarks(n1, true);
    }
    // for(int i = 0; i < path.size(); i++)
    // {
    //   bool is_goal_reached = false;
    //   goal_point[0] += actions[i][0];
    //   goal_point[1] += actions[i][1];
    //   if(i%lookAhead != 0) continue;
    //
    //   c1.set_goal(goal_point[0], goal_point[1], goal_point[2]);
    //
    //   while(!is_goal_reached)
    //   {
    //       is_goal_reached = c1.next_time_step(steer_velocities);
    //       c1.set_node_state(n1);
    //       map_obj.update_visible_landmarks(n1, true);
    //       // Sleep for 100ms
    //       std::chrono::duration<int, std::milli> sleep_time(100);
    //       std::this_thread::sleep_for(sleep_time);
    //
    //       time_elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    //   }
    // }

}



void test_()
{
    test_steer_controller();
    // test_raycast1();
    // test_raycast2();
    // test_raycast3();

    // test_controller1();
    // test_controller2();
    // test_controller3();
    // test_controller4();

    // test_map_and_controller();

    // String map_path = "data/map2.txt";

    // std::shared_ptr<MapReader> map_obj = std::make_shared<MapReader>(map_path);
    // // map_obj->visualize_map();

    // std::shared_ptr<Unique_Graph> unq_graph = std::make_shared<Unique_Graph>(map_obj, 10000, 32);

    // unq_graph->sample_vertices();

    // auto mat = unq_graph->adjacency_mat;

    // auto pt = unq_graph->knn_tree.nearest_point({1, 2, 3});

    // auto near_node = unq_graph->node_map[pt];

    // Search search;
    // point_t start{1, 4};
    // point_t goal{10, 10};
    // search.set_start(start);
    // search.set_goal(goal);

    // std::vector<point_t> path = search.plan();
    // std::cout << "Path \n";
    // for (int i = 0; i < path.size(); i++)
    // {
    //     for (int j = 0; j < path[i].size(); j++)
    //     {
    //         std::cout << path[i][j] << ' ';
    //     }
    //     std::cout << '\n';
    // }
}
