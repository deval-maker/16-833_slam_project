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
    node n1 = node(1, 400, 600, 0);

    map_obj.update_visible_landmarks(&n1, false);

    if(n1.visible_landmarks.size() == 2)
    {
        cout << "[MapReader 2] Test Passed !" << endl;
    }
    else
    {
        cout << "[MapReader 2] Test Failed !" << endl;
    }
}

void test_controller1()
{
    // Motion in X direction 
    Controller c1 = Controller();
    c1.set_current_state(400.0, 600.0, 0.0);
    c1.set_goal(406.0, 600.0, 0.0);

    bool is_goal_reached = false;
    
    while(!is_goal_reached)
    {
        is_goal_reached = c1.next_time_step();

        // Sleep for 100ms 
        std::chrono::duration<int, std::milli> sleep_time(100);
        std::this_thread::sleep_for(sleep_time);
    }
}

void test_controller2()
{
    // Motoion in theta
    Controller c1 = Controller();
    c1.set_current_state(400.0, 600.0, 0.0);
    c1.set_goal(400.0, 800.0, PI/2);

    bool is_goal_reached = false;
    
    while(!is_goal_reached)
    {
        is_goal_reached = c1.next_time_step();

        // Sleep for 100ms 
        std::chrono::duration<int, std::milli> sleep_time(100);
        std::this_thread::sleep_for(sleep_time);
    }
}


