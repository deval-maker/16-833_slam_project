#include "MapReader.h"

MapReader::MapReader(string src_path_map)
{
    string temp;
    int skip_lines = 7;
    infile.open(src_path_map);
    int line_number = 0;
    int j = 0;
    float prob = 0.0;

    while (getline(infile, temp))
    {
        line_number++;

        // Skip first n lines
        if (line_number <= 7)
        {
            continue;
        }

        stringstream ss_temp(temp);
        string intermediate;
        j = 0;
        while (getline(ss_temp, intermediate, ' '))
        {
            prob = stod(intermediate);

            if (prob <= 0)
            {
                prob = 0;
            }
            else if (prob >= 1)
            {
                prob = 1;
            }

            prob = 1 - prob;

            _map[799 - j][ 807 - line_number] = prob;

            prob = prob * 255;
            _viz_map[799 - j][ 807 - line_number] = (uint8_t)prob;
            j++;
        }
    }

    infile.close();
    std::memcpy(A.data, &(_viz_map), size_x * size_y * sizeof(uint8_t));

    cout << "Finished reading 2D map of size : (" << line_number << "," << j << ")" << endl;
}

void MapReader::visualize_map(void)
{
    cv::imshow("Particle Filter", A);
    cv::waitKey(0);
}

float *MapReader::get_prob_map(void)
{
    return &_map[0][0];
}

float MapReader::query_map(int i, int j)
{
    // j = 8000 - (j);
    if (i >= 8000 || j >= 8000 || i < 0 && j < 0)
    {
        cout << "[ Error ] Map query: " << i << " " << j << endl;
        return 1;
    }
    else
    {
        return _map[(int)i / 10][(int)j / 10];
    }
}

uint8_t *MapReader::get_viz_map(void)
{
    return &_viz_map[0][0];
}
