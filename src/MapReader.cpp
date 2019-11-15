#include "MapReader.h"

MapReader::MapReader(string src_path_map)
{
    string temp;
    infile.open(src_path_map);
    int line_number = 0;
    int j = 0;
    cout << src_path_map << endl;

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

    cout << "Finished reading 2D map of size : (" << line_number << "," << j << ")" << endl;

    std::memcpy(A.data, &(_map), size_x * size_y * sizeof(uint8_t));

}

void MapReader::visualize_map(void)
{
    cv::imshow("SLAM Project", A);
    cv::waitKey(0);
}

uint8_t MapReader::query_map(int i, int j)
{
    if (i >= MAP_SIZE_X || j >= MAP_SIZE_Y || i < 0 && j < 0)
    {
        cout << "[ Error ] Map query: " << i << " " << j << endl;
        return -1;
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
