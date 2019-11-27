#include <MapReader.h>

MapReader::MapReader(string src_path_map)
{
    string temp;
    infile.open(src_path_map);
    int line_number = 0;
    int j = 0;

    // cout << src_path_map << endl;

    while (getline(infile, temp))
    {
        stringstream ss_temp(temp);
        string intermediate;
        j = 0;
        while (getline(ss_temp, intermediate, ' '))
        {
            _map[line_number][j] = stoi(intermediate);
            j++;
        }
        line_number++;
    }

    infile.close();

    // cout << "Finished reading 2D map of size : (" << line_number << "," << j << ")" << endl;

    
    // Add Landmarks in the lookup
    point_t landmark_pose;

    // X
    landmark_pose.push_back(400);
    // Y 
    landmark_pose.push_back(900);
    // Add to lookup
    landmark_lookup.push_back(landmark_pose);

    landmark_pose.clear();

    // X
    landmark_pose.push_back(200);
    // Y 
    landmark_pose.push_back(600);
    // Add to lookup
    landmark_lookup.push_back(landmark_pose);

}

void MapReader::visualize_map(void)
{
    cv::Mat A = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8U);
    std::memcpy(A.data, &(_map), size_x * size_y * sizeof(uint8_t));
    cv::imshow("SLAM Project", A);
    cv::waitKey(0);
}

uint8_t MapReader::query_map(int i, int j)
{
    if (i >= MAP_SIZE_X || j >= MAP_SIZE_Y || i < 0 && j < 0)
    {
        cout << "[ Error ] Map query: " << i << " " << j << endl;
        return 0;
    }
    else
    {
        return _map[i][j];
    }
}

uint8_t *MapReader::get_map(void)
{
    return &_map[0][0];
}

double MapReader::correct_range(double angle)
{
    return fmod(angle, (2 * PI));
}

float MapReader::toRadian(float degree)
{
    return (degree*PI)/180;
}

void MapReader::update_visible_landmarks(node &x_t, bool visualize)
{
    double curr_orient = x_t.theta;
    curr_orient = correct_range(curr_orient);

    double start = curr_orient - toRadian(laser_fov/2);
    start = correct_range(start);

    double robox = x_t.x;
    double roboy = x_t.y;
    double angle = 0.0;

    x_t.visible_landmarks.clear();

    // cout << "Ray casting loop \n";

    Point ray_pt0 = Point(robox, roboy);
    vector<Point> rays;
    vector<Point> robot_rays;

    for (int i = 0; i < laser_fov / stepsize; i++)
    {

        angle = start + i * delta_theta;

        double newx = 0.0;
        double newy = 0.0;
        int j = 1;
        int hit_point;

        newx = robox + j * delta_dist * cos(angle);
        newy = roboy + j * delta_dist * sin(angle);

        while (true)
        {
            hit_point = query_map(newy, newx);

            if(hit_point <= total_landmarks)
            {
                if(hit_point != 0)
                {
                    // Add to the list of visible landmarks 
                    x_t.visible_landmarks.insert(hit_point);
                }

                break;
            }

            j++;
            newx = robox + j * delta_dist * cos(angle);
            newy = roboy + j * delta_dist * sin(angle);

        }

        rays.push_back(Point(newx, newy));

    }

    // cout << x_t->visible_landmarks.size() << endl;
    // cout << "Number of rays " << rays.size() << endl;

    if(visualize)
    {
        cv::Mat A = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC3);
        cv::Mat B = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC1);

        std::memcpy(B.data, _map, MAP_SIZE_X * MAP_SIZE_Y * sizeof(uint8_t));
        cv::cvtColor(B, A, cv::COLOR_GRAY2BGR);

        cv::circle(A, ray_pt0, 2, Scalar(0, 0, 255), CV_FILLED, 1, 0);

        for (int i = 0; i < rays.size(); i++)
        {
            line(A, ray_pt0, rays[i], Scalar(255, 0, 0), 1, 8);
        }

        cv::imshow("SLAM Project", A);
        cv::waitKey(10);
    }

}

vector<meas> MapReader::get_and_update_visible_landmarks(node &x_t, bool visualize)
{
    vector<meas> ret;

    double curr_orient = x_t.theta;
    curr_orient = correct_range(curr_orient);

    double start = curr_orient - toRadian(laser_fov/2);
    start = correct_range(start);

    double robox = x_t.x;
    double roboy = x_t.y;
    double angle = 0.0;

    x_t.visible_landmarks.clear();

    // cout << "Ray casting loop \n";

    Point ray_pt0 = Point(robox, roboy);
    vector<Point> rays;
    vector<Point> robot_rays;

    meas measurement;

    for (int i = 0; i < laser_fov / stepsize; i++)
    {

        angle = start + i * delta_theta;

        double newx = 0.0;
        double newy = 0.0;
        int j = 1;
        int hit_point;

        newx = robox + j * delta_dist * cos(angle);
        newy = roboy + j * delta_dist * sin(angle);

        while (true)
        {
            hit_point = query_map(newy, newx);

            if(hit_point <= total_landmarks)
            {
                if(hit_point != 0)
                {
                    measurement.landmark_id = hit_point;
                    double dist = sqrt(pow(robox - newx, 2) + pow(roboy - newy, 2));
                    measurement.dist = dist;
                    measurement.psi = angle;
                    measurement.is_visible = true;

                    if(!x_t.visible_landmarks.count(hit_point))
                    {
                        ret.push_back(measurement);
                    }
                    // Add to the list of visible landmarks 
                    x_t.visible_landmarks.insert(hit_point);
                }

                break;
            }

            j++;
            newx = robox + j * delta_dist * cos(angle);
            newy = roboy + j * delta_dist * sin(angle);

        }

        rays.push_back(Point(newx, newy));

    }

    // cout << x_t->visible_landmarks.size() << endl;
    // cout << "Number of rays " << rays.size() << endl;

    if(visualize)
    {
        cv::Mat A = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC3);
        cv::Mat B = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC1);

        std::memcpy(B.data, _map, MAP_SIZE_X * MAP_SIZE_Y * sizeof(uint8_t));
        cv::cvtColor(B, A, cv::COLOR_GRAY2BGR);

        cv::circle(A, ray_pt0, 2, Scalar(0, 0, 255), CV_FILLED, 1, 0);

        for (int i = 0; i < rays.size(); i++)
        {
            line(A, ray_pt0, rays[i], Scalar(255, 0, 0), 1, 8);
        }

        cv::imshow("SLAM Project", A);
        cv::waitKey(10);
    }

    return ret;
}

point_t MapReader::get_landmark_pose(int landmark_id)
{
    point_t ret;

    if(landmark_id > landmark_lookup.size())
    {
        ret.push_back(-1);
        ret.push_back(-1);
        cout << "[Error MapReader] Wrong query !" << endl; 

    }
    else
    {
        ret = landmark_lookup[landmark_id-1];
    }

    return ret;

}

meas MapReader::get_measurement(int landmark_id, Eigen::Vector3f curr_pose)
{
    bool visualize = false;
    meas ret;
    ret.is_visible = false;

    double curr_orient = curr_pose[2];
    curr_orient = correct_range(curr_orient);

    double robox = curr_pose[0];
    double roboy = curr_pose[1];
    double angle = 0.0;

    point_t land_pose;
    land_pose = get_landmark_pose(landmark_id);

    // double start = curr_orient - toRadian(laser_fov/2);
    double start = atan2(land_pose[1] - roboy, land_pose[0]-robox);
    start = correct_range(start);

    Point ray_pt0 = Point(robox, roboy);
    vector<Point> rays;

    for (int i = 0; i < 1; i++)
    {

        angle = start + i * delta_theta;

        double newx = 0.0;
        double newy = 0.0;
        int j = 1;
        int hit_point;

        newx = robox + j * delta_dist * cos(angle);
        newy = roboy + j * delta_dist * sin(angle);

        while (true)
        {
            hit_point = query_map(newy, newx);

            if(hit_point <= total_landmarks)
            {
                if(hit_point != 0)
                {
                    ret.landmark_id = hit_point;
                    double dist = sqrt(pow(robox - newx, 2) + pow(roboy - newy, 2));
                    ret.dist = dist;
                    ret.psi = angle;
                    ret.is_visible = true;
                }

                break;
            }

            j++;
            newx = robox + j * delta_dist * cos(angle);
            newy = roboy + j * delta_dist * sin(angle);

        }
        rays.push_back(Point(newx, newy));
    }

    if(visualize)
    {
        cv::Mat A = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC3);
        cv::Mat B = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC1);

        std::memcpy(B.data, _map, MAP_SIZE_X * MAP_SIZE_Y * sizeof(uint8_t));
        cv::cvtColor(B, A, cv::COLOR_GRAY2BGR);

        cv::circle(A, ray_pt0, 2, Scalar(0, 0, 255), CV_FILLED, 1, 0);

        for (int i = 0; i < rays.size(); i++)
        {
            line(A, ray_pt0, rays[i], Scalar(255, 0, 0), 1, 8);
        }

        cv::imshow("SLAM Project", A);
        cv::waitKey(0);
    }

    if(ret.landmark_id != landmark_id)
    {   
        ret.is_visible = false;
        cout << "[MapReader] something is seriously wrong" << endl;
    }

    return ret;
}

