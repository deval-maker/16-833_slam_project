#include <test.h>

void test_raycast1()
{
    String map_path = "data/map1.txt";
    MapReader map_obj = MapReader(map_path);
    
    map_obj.visualize_map();

    cout << "[MapReader 1] Test Passed !" << endl;
}

void test_raycast2()
{
    String map_path = "data/map2.txt";
    MapReader map_obj = MapReader(map_path);
    
    // Make sure that the laser_fov is 360  
    node n1 = node(1, 400, 600, 0);

    map_obj.update_visible_landmarks(&n1, true);

    if(n1.visible_landmarks.size() == 2)
    {
        cout << "[MapReader 2] Test Passed !" << endl;
    }
    else
    {
        cout << "[MapReader 2] Test Failed !" << endl;
    }
}