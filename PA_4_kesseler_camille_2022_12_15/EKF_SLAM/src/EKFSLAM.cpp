#include "EKFSLAM.h"

namespace EKF_SLAM
{
    // Initialisation function 
    MY_EKF::MY_EKF(const Pose &Ini_pose,const Map &Ini_map){
        
        int dim=3+3*Ini_map.data.size();
        state.mean = Eigen::VectorXf::Zero(dim);
        state.covariance=Eigen::MatrixXf::Zero(dim,dim);
        
        //robot position initialisation 
        state.mean(0)=Ini_pose.x;
        state.mean(1)=Ini_pose.y;
        state.mean(2)=Ini_pose.theta;
         Eigen::Matrix3f cov; 
            cov<<0, 0, 0,
             0, 1, 0,
             0, 0, 1;
        state.covariance.block<3,3>(0,0)=Ini_pose.covariance;
        //state.covariance.block<3,3>(0,0)=cov;

        //landmark position initialisation

        for (int i=0;i<(int)Ini_map.data.size();i++){
            
            state.mean(3+3*i)=Ini_map.data[i].id;
            state.mean(4+3*i)=Ini_map.data[i].x;
            state.mean(5+3*i)=Ini_map.data[i].y;
           
            state.covariance.block<3,3>(3+3*i,3+3*i)=cov;
            Landmark_detection.push_back(0); 
        }
       
        map=Ini_map;
        Trajectory.push_back(Ini_pose);

        
        //cout<<state.mean<<endl;
        //cout<<state.covariance<<endl;

    }
    //////////////////////////////////////////////////////////////
        // Function definition 

        int MotionModel(
        // motion model function 
        const State &state,
        const Odometry &movement,
        Eigen::Vector3f &g
        ){
            g(0)=state.mean(0)+movement.t*cos(state.mean(2)+movement.r1);
            g(1)=state.mean(1)+movement.t*sin(state.mean(2)+movement.r1);
            g(2)=state.mean(2)+movement.r1+movement.r2; 

            return 0; 
        }
        
        int MotionModelDerivative(
        // motion model derivative function 
        const State &state,
        const Odometry &movement,
        Eigen::Matrix3f &G
        ){
            G=Eigen::Matrix3f::Identity();
            G(0,2)=-movement.t*sin(state.mean(2)+movement.r1);
            G(1,2)=movement.t*cos(state.mean(2)+movement.r1);
            
            return 0; 
        }

        int ObservationModel(
        // Observation model function 
        const State &state,
        const int id_position,
        Eigen::Vector3f &h
        ){
            h(0)=state.mean(3*id_position);
            //cout<<"used for computation"<<state.mean<<endl;
            h(1)=sqrt((state.mean(1+3*id_position)-state.mean(0))*(state.mean(1+3*id_position)-state.mean(0))+(state.mean(2+3*id_position)-state.mean(1))*(state.mean(2+3*id_position)-state.mean(1)));
            float dd=atan2((state.mean(2+3*id_position)-state.mean(1)),(state.mean(1+3*id_position)-state.mean(0)))-state.mean(2);
            if(dd<-M_PI){
                 h(2)=dd+2*M_PI;
            }
            else if(dd>M_PI){
                 h(2)=dd-2*M_PI;}
            else h(2)=dd;
          
            return 0; 
        }
    
        int ObservationModelDerivative(
        // Observation model derivative function 
        const State &state,
        const int id_position,
        Eigen::MatrixXf &H
        ){

            float distance=(state.mean(1+3*id_position)-state.mean(0))*(state.mean(1+3*id_position)-state.mean(0))+(state.mean(2+3*id_position)-state.mean(1))*(state.mean(2+3*id_position)-state.mean(1)); 
            
            H(0,0)=0;
            H(0,1)=0;
            H(0,2)=0;
            H(0,3)=1;
            H(0,4)=0;
            H(0,5)=0;

            H(1,0)=-(state.mean(1+3*id_position)-state.mean(0))/sqrt(distance);
            H(1,1)=-(state.mean(2+3*id_position)-state.mean(1))/sqrt(distance);
            H(1,2)=0;
            H(1,3)=0;
            H(1,4)=(state.mean(1+3*id_position)-state.mean(0))/sqrt(distance);
            H(1,5)=(state.mean(2+3*id_position)-state.mean(1))/sqrt(distance);
            
            H(2,0)=(state.mean(2+3*id_position)-state.mean(1))/distance;
            H(2,1)=-(state.mean(1+3*id_position)-state.mean(0))/distance;
            H(2,2)=-1;
            H(2,3)=0;
            H(2,4)=-(state.mean(2+3*id_position)-state.mean(1))/distance;
            H(2,5)=(state.mean(1+3*id_position)-state.mean(0))/distance;

            return 0; 
        }



   ////////////////////////////////////////////////////////////
        // Public function

        int MY_EKF::predict(
            const Odometry &Movement,
            const Eigen::Matrix3f &R){
            
            
            //state mean prediction computation
            Eigen::Vector3f g; 
            MotionModel(state,Movement,g);
            //cout<<g<<endl;
            state.mean[0]=g(0);
            state.mean[1]=g(1);
            state.mean[2]=g(2);
    
            //state covariance prediction computation 
            Eigen::Matrix3f G;
            MotionModelDerivative(state,Movement,G);
            Eigen::MatrixXf G_full=Eigen::MatrixXf::Identity(state.covariance.rows(),state.covariance.cols()); 
            G_full.block<3,3>(0,0)=G; 
            Eigen::MatrixXf F=Eigen::MatrixXf::Zero(state.mean.size(),3);
            F.block<3,3>(0,0)=Eigen::Matrix3f::Identity(); 
            state.covariance=G_full*state.covariance*G_full.transpose()+F*R*F.transpose(); 

            
            //cout<<state.covariance<<endl;
            return 0; 
        }

        int MY_EKF::correct( 
            const vector<Landmark> &lm,
            const Eigen::Matrix3f &Q
        ){
           
            // loop over the landmark measurement
            Eigen::MatrixXf H_full=Eigen::MatrixXf::Zero(3*lm.size(),state.mean.size());
            Eigen::MatrixXf Q_full=Eigen::MatrixXf::Zero(3*lm.size(),3*lm.size()); 
            Eigen::VectorXf h_full=Eigen::VectorXf::Zero(3*lm.size());
            Eigen::VectorXf z_full=Eigen::VectorXf::Zero(3*lm.size()); 
            
            for (int k=0;k<(int)lm.size();k++)
            {
            Landmark landmark_measurement=lm[k];    
            //cout<<Landmark_detection[landmark_measurement.id-1]<<endl;
            if(Landmark_detection[landmark_measurement.id-1]==0){
                //Initialised the landmark with the measurement when first detection
                state.mean(1+3*landmark_measurement.id)=state.mean(0)+landmark_measurement.range*cos(state.mean(2)+landmark_measurement.bearing);
                state.mean(2+3*landmark_measurement.id)=state.mean(1)+landmark_measurement.range*sin(state.mean(2)+landmark_measurement.bearing);
                //state.covariance.block<3,3>(3*landmark_measurement.id,3*landmark_measurement.id)=state.covariance.block<3,3>(0,0);

            }
            //cout<<"Initial_position"<<state.mean(4+3*landmark_measurement.id)<<" ; "<<state.mean(5+3*landmark_measurement.id)<<endl;
            Landmark_detection[landmark_measurement.id-1]+=1;
            
            //Observation model
            Eigen::Vector3f h;  
            ObservationModel(state,landmark_measurement.id,h);
            h_full.segment<3>(3*k)=h;

            z_full(3*k)=landmark_measurement.id;
            z_full(1+3*k)=landmark_measurement.range;
            z_full(2+3*k)=landmark_measurement.bearing;
           
            //Derivative of the observation model
            Eigen::MatrixXf H=Eigen::MatrixXf::Zero(3,6);
            ObservationModelDerivative(state,landmark_measurement.id,H);
            
            Eigen::MatrixXf F=Eigen::MatrixXf::Zero(3*lm.size(),3);
            F.block<3,3>(3*k,0)=Eigen::Matrix3f::Identity();
            Eigen::MatrixXf P=Eigen::MatrixXf::Zero(6,state.mean.size());
            P.block<3,3>(0,0)=Eigen::Matrix3f::Identity();
            P.block<3,3>(3,3*landmark_measurement.id)=Eigen::MatrixXf::Identity(3,3);
            H_full+=F*H*P;
    
            // noise model
            Q_full.block<3,3>(k*3,k*3)=Q;
            
            }
            
            //cout<<"measurement"<<z_full<<endl;
            //cout<<"computed"<<h_full<<endl;
            //cout<<"error"<<z_full-h_full<<endl;

            //Kalman gain computation
            Eigen::MatrixXf K;
            K=state.covariance*H_full.transpose()*(H_full*state.covariance*H_full.transpose()+Q_full).inverse();

            //cout<<K<<endl;
            // state mean corrected computation 
            
            //cout<<"update_step"<<K*(z_full-h_full)<<endl;
            state.mean=state.mean+K*(z_full-h_full);
            // state covariance corrected computation 
            
            state.covariance=(Eigen::MatrixXf::Identity(state.mean.size(),state.mean.size())-K*H_full)*state.covariance;

            //cout<<state.mean<<endl;
            //cout<<state.covariance.block<9,9>(0,0)<<endl;
            
            return 0;
        }


        int MY_EKF::UpdateTraj(){
            //update the trajectory
            Pose robot;
            robot.x=state.mean(0);
            robot.y=state.mean(1);
            robot.theta=state.mean(2);
            robot.covariance=state.covariance.block<3,3>(0,0);
            Trajectory.push_back(robot);

            //update map
            for (int i=0;i<(int)map.data.size();i++)
            {
                map.data[i].x=state.mean(4+3*i);
                map.data[i].y=state.mean(5+3*i);
            }
            
            return 0; 
        }

        int MY_EKF::SaveMapPoints(string file){
            ofstream output_file(file);
            if(output_file.is_open()){
                for(int i=0; i<(int)map.data.size();i++)
                {
                 output_file<<to_string(map.data[i].id)<<" "<<to_string(map.data[i].x)<<" "<<to_string(map.data[i].y)<<endl;  

                }

            }
            else{
                exit(EXIT_FAILURE);
                cout<<"cannot write map data"<<endl;
            }

            return 0; 
        } 

        int MY_EKF::SaveTrajectory(string file){
            ofstream output_file(file);
            if(output_file.is_open()){
                for(int i=0; i<(int)Trajectory.size();i++)
                {
                 output_file<<to_string(Trajectory[i].x)<<" "<<to_string(Trajectory[i].y)<<" "<<to_string(Trajectory[i].theta)<<" ";
                 output_file<<to_string(Trajectory[i].covariance(0))<<" ";
                 output_file<<to_string(Trajectory[i].covariance(1))<<" ";
                 output_file<<to_string(Trajectory[i].covariance(2))<<" ";
                 output_file<<to_string(Trajectory[i].covariance(3))<<" ";
                 output_file<<to_string(Trajectory[i].covariance(4))<<" ";
                 output_file<<to_string(Trajectory[i].covariance(5))<<" ";
                 output_file<<to_string(Trajectory[i].covariance(6))<<" ";
                 output_file<<to_string(Trajectory[i].covariance(7))<<" ";
                 output_file<<to_string(Trajectory[i].covariance(8))<<endl;

                }

            }
            else{
                exit(EXIT_FAILURE);
                cout<<"cannot write trajectory data"<<endl;
            }

            return 0; 
        }

        int MY_EKF::save_instant(int step){
            string name_file="../plot/Instant_dat/step";
            name_file+=to_string(step);
            name_file+=".dat";
            ofstream output_file(name_file);
            if(output_file.is_open()){
                output_file<<to_string(Trajectory.back().x)<<" "<<to_string(Trajectory.back().y)<<" "<<to_string(Trajectory.back().theta)<<endl;
                for(int i=0; i<(int)map.data.size();i++)
                {
                 output_file<<to_string(map.data[i].id)<<" "<<to_string(map.data[i].x)<<" "<<to_string(map.data[i].y)<<endl;  

                }
            }
            else{
                exit(EXIT_FAILURE);
                cout<<"cannot write trajectory data"<<endl;
            }

            return 0; 
        } 

        int  MY_EKF::RMSE_land(Map GT_map){
            
            double rmse=0; 
            for (int i=0; i<(int)map.data.size();i++){
                // Check if the landmark was detected
                if(Landmark_detection[i]!=0){
                    //Check if ground_truth data present
                    if(GT_map.isIn(map.data[i].id)){
                        MapPoint GT_land=GT_map.getMapPoint(map.data[i].id); 
                        rmse+=(GT_land.x-map.data[i].x)*(GT_land.x-map.data[i].x)+(GT_land.y-map.data[i].y)*(GT_land.y-map.data[i].y);

                    }
                }


            }
            RMSE.push_back(rmse);

            return 0;
        }


        int MY_EKF::SaveRMSE(string file){

            ofstream output_file(file);
            if(output_file.is_open()){
                for(int i=0; i<(int)RMSE.size();i++)
                {
                 output_file<<to_string(i+1)<<" "<<to_string(RMSE[i])<<endl;  

                }

            }
            else{
                exit(EXIT_FAILURE);
                cout<<"cannot write map data"<<endl;
            }



            return 0; 
        }



}