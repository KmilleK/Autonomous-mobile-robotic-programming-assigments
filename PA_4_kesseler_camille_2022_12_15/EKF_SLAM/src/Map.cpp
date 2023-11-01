#include "Map.h"

namespace EKF_SLAM
{
    Map::Map(string input_dir)
    {
        ifstream input_file(input_dir);
        if (input_file.is_open())
        {
            while (input_file)
            {
                string s;
                getline(input_file, s);
                
                if(s=="")
                    break;

                vector<string> answer;
                stringstream ss(s);
                string temp;

                MapPoint mp;
                
                while (getline(ss, temp, ' '))
                {
                    answer.push_back(temp);
                }

                mp.id=stol(answer[0]);
                mp.x=stof(answer[1]);
                mp.y=stof(answer[2]);

                data.push_back(mp);
            }
        }
        else{
            exit(EXIT_FAILURE);
            cout<<"cannot read map data"<<endl;
        }
    }

    bool Map::isIn(long id){
        for (int lm=0;lm<(int)data.size();lm++){
            if (data[lm].id==id) return true;
        }
        return false; 

    }

    MapPoint Map::getMapPoint(long id){
        for (int lm=0;lm<(int)data.size();lm++){
            if (data[lm].id==id) return data[lm];
        }

    }

    int Map::addMapPoint(MapPoint landmark){
        data.push_back(landmark);
    }
}