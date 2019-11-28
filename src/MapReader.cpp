#include <MapReader.h>
#include <utils.h>
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
    landmark_pose.push_back(200);
    // Add to lookup
    landmark_lookup.push_back(landmark_pose);

    // X
    landmark_pose[0] = 500;
    // Y
    landmark_pose[1] = 500;
    // Add to lookup
    landmark_lookup.push_back(landmark_pose);
    // X
    landmark_pose[0] = 400;
    // Y
    landmark_pose[1] = 900;
    // Add to lookup
    landmark_lookup.push_back(landmark_pose);
    // X
    landmark_pose[0] = 900;
    // Y
    landmark_pose[1] = 400;
    // Add to lookup
    landmark_lookup.push_back(landmark_pose);
    // X
    landmark_pose[0] = 100;
    // Y
    landmark_pose[1] = 400;
    // Add to lookup
    landmark_lookup.push_back(landmark_pose);
}

void MapReader::visualize_map(void)
{
    clear_session();
    viz_session();
}

void MapReader::visualize_point(point_t point, cv::viz::Color color)
{
    Point ray_pt0 = Point((int)point[0], (int)point[1]);
    cv::circle(A, ray_pt0, 5, color, CV_FILLED, 1, 0);
}

void MapReader::visualize_path(vector<point_t> path, cv::viz::Color color)
{
    Point pt;

    for(int i = 0; i < path.size(); i++)
    {
        pt = Point(path[i][0], path[i][1]);
        cv::circle(A, pt, 5, color, CV_FILLED, 1, 0);
    }
}
double to_degree(double radian)
{
    return radian*PI/180;
}

void MapReader::visualize_ellipse(Eigen::Vector2f mean, Eigen::Matrix2f sigma)
{
    std::cout<<"Sigma \n";
    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            std::cout<<sigma(i,j)<<" ";            
        }
        std::cout<<"\n";
    }
    std::cout<<"\n\n";

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver(sigma);
    if (eigensolver.info() != Eigen::Success)
        {
            std::cout<<"[ERROR] Problem with Eigen vector computation \n";
            return ;
        }

    // find angle of eigvector fo largest eigen value
    double theta = atan2(double(eigensolver.eigenvectors()(1, 0)), double(eigensolver.eigenvectors()(0, 0)));
    // cout << "theta of ellipse" << theta << endl;
    double factor = 1;
    double a = eigensolver.eigenvalues()(1,0); 
    // if(isnan(a) || a <= 0)
    // {
    //     std::cout<<"A is nan \n";
    //     a = 0;
    // }
    double b = eigensolver.eigenvalues()(0,0); 
    // if(isnan(b) || b <= 0)
    // {
    //     std::cout<<"B is nan \n";
    //     b = 0;
    // }
    std::cout<<"Axes length "<<factor*sqrt(0.5991*a)<<" "<<factor*sqrt(0.5991*b)<<'\n';

    cv::ellipse(A, Point(mean[0], mean[1]), Size(factor*sqrt(0.5991*a), factor*sqrt(0.5991*b)), to_degree(theta), 0, 360, Scalar(255,0,0), 3, 8);
}

void MapReader::clear_session()
{
    A = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC3);
    B = Mat(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC1);

    std::memcpy(B.data, _map, MAP_SIZE_X * MAP_SIZE_Y * sizeof(uint8_t));
    cv::cvtColor(B, A, cv::COLOR_GRAY2BGR);
}

void MapReader::viz_session()
{
    cv::imshow("SLAM Project", A);
    cv::waitKey(0);
}

void MapReader::visualize_UG(vector<node> ug, cv::viz::Color color)
{
    Point pt;

    for(int i = 0; i < ug.size(); i++)
    {
        pt = Point(ug[i].x, ug[i].y);
        cv::circle(A, pt, 2, color, CV_FILLED, 1, 0);
    }

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
double wrapMax_util(double x, double max)
{
    return fmod(max + fmod(x, max), max);
}
double wrap2pi_util(double x)
{
    return -M_PI + wrapMax_util(x + (M_PI), 2*M_PI );
}

vector<meas> MapReader::get_landmark_measurement(Eigen::Vector3f curr_pose)
{
    vector<meas> ret;
    std::unordered_set<int> visible_landmarks;
    bool visualize = false;

    double curr_orient = curr_pose[2];
    curr_orient = correct_range(curr_orient);

    double start = curr_orient - toRadian(laser_fov/2);
    start = correct_range(start);

    double robox = curr_pose[0];
    double roboy = curr_pose[1];
    double angle = 0.0;

    visible_landmarks.clear();

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
                    measurement.psi = wrap2pi_util(angle);
                    measurement.is_visible = true;

                    if(!visible_landmarks.count(hit_point))
                    {
                        ret.push_back(measurement);
                    }
                    // Add to the list of visible landmarks
                    visible_landmarks.insert(hit_point);
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
