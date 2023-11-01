////////////////////////////////////////////////////////////
// This C++ program implements simple 2D EKF SLAM with example data.
// Date: 2022/11/29
// Author: Jiseong CHUNG

#include "Map.h"
#include "Measurement.h"
#include "EKFSLAM.h"


using namespace std;
using namespace EKF_SLAM;

bool valid_argument(int argc, char* argv[])
{
    if (argc==3)
        return true;
    else {
        cout<<"Usage: ./test_EKF {PATH_TO_WORLD_DAT}/world.dat {PATH_TO_SENSOR_DAT}/sensor_data.dat"<<endl;
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char* argv[])
{


    
    if(valid_argument(argc, argv))
    {
      
        // argument setting
        string world_input = argv[1];
        string sensor_input = argv[2];

        Map map(world_input);
        Measurement measurement(sensor_input);
        

        cout<<"Data Reading Done"<<endl;
        
        //Initialize the EKFSLAM class 
        Pose Initial_state;
        Initial_state.x=0;
        Initial_state.y=0;
        Initial_state.theta=0;
        Initial_state.covariance=Eigen::MatrixXf::Zero(3,3); 

        Map Initial_map;
        for (int i=0;i<(int)map.data.size();i++)
        {   
            MapPoint ini;
            ini.id=i+1;
            ini.x=0;
            ini.y=0;
            Initial_map.data.push_back(ini);
        }
        MY_EKF ekf_slam(Initial_state,Initial_map); 
        cout<<"created init"<<endl;
    
        // matrix for the noise 
        Eigen::Matrix3f Rt;
        Rt<< 0.1, 0, 0,
             0, 0.1, 0,
             0, 0, 0.01;
        Eigen::Matrix3f Qt;
        Qt=Rt;

        //Iterate though all the measurement 
        //(int)measurement.data.size()
        for (int step=0; step<(int)measurement.data.size();step++)
        {
            
            //Prediction step 
            ekf_slam.predict(measurement.data[step].odom,Rt);
            //cout<<ekf_slam.state.mean<<endl;

            //Correction step 
            
            ekf_slam.correct(measurement.data[step].lm,Qt);
            //cout<<ekf_slam.state.mean<<endl;
            
            //Update trajectory

            ekf_slam.UpdateTraj();
            

            //Compute RMSE for the landmark 
            ekf_slam.RMSE_land(map);
            
            //Store instant data in a dat file 
            ekf_slam.save_instant(step); 

        }

        cout<<"EKF done with:"<<ekf_slam.Trajectory.size()<<" steps"<<endl;
 
        

        //Plot the results => save in dat file format for reading later in matlab 

        ekf_slam.SaveMapPoints("MyMap.dat");
        ekf_slam.SaveTrajectory("MyTraj.dat"); 
        ekf_slam.SaveRMSE("MyRMSE.dat");
        

        //////////////////////////////////////
    }
}