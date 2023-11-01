#include "Measurement.h"

namespace EKF_SLAM
{
    Measurement::Measurement(string input_dir)
    {
        ifstream input_file(input_dir);
        if (input_file.is_open())
        {
            Sensor record;

            while (input_file)
            {
                string s;
                getline(input_file, s);

                if (s == "")
                    break;

                vector<string> answer;
                stringstream ss(s);
                string temp;

                while (getline(ss, temp, ' '))
                {
                    answer.push_back(temp);
                }

                if (answer[0] == "ODOMETRY")
                {
                    if (record.lm.size() != 0)
                    {
                        data.push_back(record);
                        record.lm.clear();
                    }

                    Odometry temp_odom;
                    temp_odom.r1 = stof(answer[1]);
                    temp_odom.t = stof(answer[2]);
                    temp_odom.r2 = stof(answer[3]);
                    record.odom = temp_odom;
                }
                else if (answer[0] == "SENSOR")
                {
                    Landmark temp_lm;
                    temp_lm.id = stoi(answer[1]);
                    temp_lm.range = stof(answer[2]);
                    temp_lm.bearing = stof(answer[3]);
                    record.lm.push_back(temp_lm);
                }
            }

            if (record.lm.size() != 0)
            {
                data.push_back(record);
            }
        }
        else{
            exit(EXIT_FAILURE);
            cout<<"cannot read map data"<<endl;
        }
    }
}