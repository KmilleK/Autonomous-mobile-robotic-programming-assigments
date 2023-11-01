#include "Reader.h"
//Reader file obj
namespace BA
{
    Reader::Reader(string data) : data_path(data) {}

    Mat33 Reader::init_K()
    {
        string camera_input = data_path + "camera.csv";
        Mat33 K;
        int i=0;

        ifstream input_file(camera_input);
        if (input_file.is_open())
        {
            while (input_file)
            {
                string s;
                getline(input_file, s);
                
                if(s=="")
                    break;
                if(s.substr(0,1)=="#")
                    continue;

                vector<string> answer;
                stringstream ss(s);
                string temp;
                
                while (getline(ss, temp, ','))
                {
                    answer.push_back(temp);
                }
                K.row(i) = Vec3(stod(answer[0]), stod(answer[1]),stod(answer[2]));
                i++;
            }

            return K;
        }
        else
            exit(EXIT_FAILURE);
    }

    vector<Frame> Reader::init_frame()
    {
        string pose_input = data_path + "camera_poses.csv";
        vector<Frame> whole_frame;
        int i=0;

        ifstream input_file(pose_input);
        if (input_file.is_open())
        {
            while (input_file)
            {
                string s;
                getline(input_file, s);
                
                if(s=="")
                    break;
                if(s.substr(0,1)=="#")
                    continue;

                vector<string> answer;
                stringstream ss(s);
                string temp;
                
                while (getline(ss, temp, ','))
                {
                    answer.push_back(temp);
                }
                Eigen::Quaterniond Q(stod(answer[0]),stod(answer[1]),stod(answer[2]),stod(answer[3]));
                Vec3 tvec = Vec3(stod(answer[4]),stod(answer[5]),stod(answer[6]));

                Frame one_frame(i,SE3(Q,tvec));
                whole_frame.push_back(one_frame);
                i++;
            }

            return whole_frame;
        }
        else
            exit(EXIT_FAILURE);
    }

    vector<vector<Feature>> Reader::init_observation()
    {
        string keypoints_input = data_path + "keypoints.csv";
        vector<vector<Feature>> whole_observation;

        ifstream input_file(keypoints_input);
        if (input_file.is_open())
        {
            while (input_file)
            {
                string s;
                getline(input_file, s);
                
                if(s=="")
                    break;
                if(s.substr(0,1)=="#")
                    continue;

                vector<string> answer;
                stringstream ss(s);
                string temp;
                
                while (getline(ss, temp, ','))
                {
                    answer.push_back(temp);
                }

                int num_observation=stoi(answer[0])/2;
                vector<Feature> one_frame;

                for(int i=0;i<num_observation;i++)
                {
                    Vec2 pt_;
                    pt_[0]=stod(answer[2*i+1]);
                    pt_[1]=stod(answer[2*i+2]);
                    one_frame.push_back(pt_);
                }

                whole_observation.push_back(one_frame);
            }

            return whole_observation;
        }
        else
            exit(EXIT_FAILURE);
    }

    vector<vector<int>> Reader::init_ob_ids()
    {
        string ob_ids_input = data_path + "point_ids.csv";
        vector<vector<int>> point_id;

        ifstream input_file(ob_ids_input);
        if (input_file.is_open())
        {
            while (input_file)
            {
                string s;
                getline(input_file, s);
                
                if(s=="")
                    break;
                if(s.substr(0,1)=="#")
                    continue;

                vector<string> answer;
                stringstream ss(s);
                string temp;
                
                while (getline(ss, temp, ','))
                {
                    answer.push_back(temp);
                }

                int num_observation=stoi(answer[0]);
                vector<int> one_frame;

                for(int i=0;i<num_observation;i++)
                {
                    one_frame.push_back(stoi(answer[i+1]));
                }

                point_id.push_back(one_frame);
            }

            return point_id;
        }
        else
            exit(EXIT_FAILURE);
    }

    vector<MapPoint> Reader::init_point()
    {
        string point_input = data_path + "points.csv";
        vector<MapPoint> mp;
        int i=0;

        ifstream input_file(point_input);
        if (input_file.is_open())
        {
            while (input_file)
            {
                string s;
                getline(input_file, s);
                
                if(s=="")
                    break;
                if(s.substr(0,1)=="#")
                    continue;

                vector<string> answer;
                stringstream ss(s);
                string temp;
                
                while (getline(ss, temp, ','))
                {
                    answer.push_back(temp);
                }

                Vec3 pt_;
                pt_[0]=stof(answer[0]);
                pt_[1]=stof(answer[1]);
                pt_[2]=stof(answer[2]);
                MapPoint one_point(pt_);
                one_point.id_=i;

                mp.push_back(one_point);
                i++;
            }

            return mp;
        }
        else
            exit(EXIT_FAILURE);
    }
}
