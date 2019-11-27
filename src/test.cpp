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

    land_meas = map_obj.get_landmark_measurements(curr_pose);

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
        is_goal_reached = c1.next_time_step();

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
        is_goal_reached = c1.next_time_step();

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
        is_goal_reached = c1.next_time_step();

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
        is_goal_reached = c1.next_time_step();

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
        is_goal_reached = c1.next_time_step();
        
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
