/*
    This program implements Bundle adjustment with simple example data.
    Date: 2022/12/16
    Author: Jiseong CHUNG
*/

#include "Common.h"
#include "Frame.h"
#include "Feature.h"
#include "Reader.h"
#include "MapPoint.h"
#include "ProjectionFactor.h"

#include <ceres/ceres.h>

using namespace BA;

bool valid_argument(int argc, char* argv[])
{
    if (argc==2)
        return true;
    else {
        cout<<"check your argument(data path)"<<endl;
        exit(EXIT_FAILURE);
    }
}

int RMSE_compute(
    const vector<Frame> &Opti_frame,
    double &RMSE_tot,
    vector<double> &RMSE_details
)
{
    // Get Ground truth data 
    Reader input_GT("../data/data_gnd/");
    vector<Frame> GT_frame = input_GT.init_frame();

    //Compute the RMSE 
    double rmse = 0;

    for (int i = 0; i < (int)Opti_frame.size(); i++) {

        double error = (GT_frame[i].pose_.inverse()* Opti_frame[i].pose_).log().norm();
        rmse += error * error;
        RMSE_details.push_back(error);
    }
    rmse = rmse / double(Opti_frame.size());
    RMSE_tot= sqrt(rmse);
    cout << "RMSE = " << RMSE_tot << endl;

    return 0; 
}

int main(int argc, char** argv)
{
    
    if(valid_argument(argc,argv))
    {
        cout<<"Running"<<endl;
        // initialize dataset
        Reader input(argv[1]);
        Mat33 K = input.init_K();
        vector<Frame> frame = input.init_frame();
        vector<vector<Feature>> observation = input.init_observation();
        vector<vector<int>> observation_ids = input.init_ob_ids();
        vector<MapPoint> map_point = input.init_point();
    
        cout<<"Reading Data done"<<endl;
        
        // plot point 1 rotation 

        //Vec3 rotated=frame[0].world2camera(map_point[0].pose_);
        //cout<<rotated; 
        // set ceres solver options
        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = false;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.num_threads = 4;
        options.max_num_iterations = 10;
        options.max_solver_time_in_seconds = 0.04;

        //cout<<"initial frame 1"<<frame[0].pose_<<endl;

        //Saving parameters in the correct mutable form for ceres solving 

        const int nb_of_frame=(int)frame.size();
        const int nb_of_landmark=(int)map_point.size();
       
        // Parameters block to solve: [Frame0,... ,FrameN,Point0,...,PointN ] 
        double Parameters[7*nb_of_frame+3*nb_of_landmark];
        // Fill with the initial information
        for (int i=0;i<nb_of_frame;i++)
        {
            // translation part 
            Parameters[7*i]=frame[i].pose_.params()[4];
            Parameters[7*i+1]=frame[i].pose_.params()[5];
            Parameters[7*i+2]=frame[i].pose_.params()[6];
            //quaternion part 
            Parameters[7*i+3]=frame[i].pose_.params()[3];
            Parameters[7*i+4]=frame[i].pose_.params()[0];
            Parameters[7*i+5]=frame[i].pose_.params()[1];
            Parameters[7*i+6]=frame[i].pose_.params()[2];
        }

        for (int i=0;i<nb_of_landmark;i++){

            Parameters[7*nb_of_frame+3*i]=map_point[i].pose_[0];
            Parameters[7*nb_of_frame+3*i+1]=map_point[i].pose_[1];
            Parameters[7*nb_of_frame+3*i+2]=map_point[i].pose_[2];
        }

        // Loop over the Frame
        for (int frame_id=0;frame_id<(int)frame.size();frame_id++)
        {
            //Frame pointer localisation
            double* tvec= Parameters+frame_id*7;
            double* qvec= Parameters+frame_id*7+3;
            
            vector<Feature> frame_keypoints =observation[frame_id];

            //loop over the Keypoints detected in the frame
            for (int i=0;i<(int)frame_keypoints.size();i++){
                
                //Read Data for the keypoint
                Feature actual_keypoint =frame_keypoints[i];
                int actual_keypoint_id =observation_ids[frame_id][i];
                //cout<<actual_keypoint.pt_<<endl;
                double* landmark=Parameters+nb_of_frame*7+actual_keypoint_id*3;
                //cout<<landmark<<endl;
                //Create cost function 
                ceres::CostFunction *cost_function; 

                cost_function = ProjectionFactor::Create(actual_keypoint.pt_,K);

                // Add Residual block
               
                //problem.AddResidualBlock(cost_function,nullptr,qvec,tvec,landmark);
                problem.AddResidualBlock(cost_function,new ceres::CauchyLoss(0.5),qvec,tvec,landmark);
                problem.SetParameterBlockConstant(landmark);
                
            }


        }
        
        
        std::cout << "Solving ceres BA ... " << endl;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        //std::cout<<Parameters[0]<<endl;

        // RMSE computation 
        vector<Frame> Opti_frame; 
        for (int i=0;i<nb_of_frame;i++)
        {
            Eigen::Quaterniond Q(Parameters[7*i+3],Parameters[7*i+4],Parameters[7*i+5],Parameters[7*i+6]);
            Vec3 tvec = Vec3(Parameters[7*i],Parameters[7*i+1],Parameters[7*i+2]);

            Frame one_frame(i,SE3(Q,tvec));
            Opti_frame.push_back(one_frame);
        }

        double RMSE_tot;
        vector<double> RMSE_details;
        RMSE_compute(Opti_frame,RMSE_tot,RMSE_details);

        // Plot to do 
        std::ofstream myfile;
        myfile.open ("../data/data_opti/camera_poses.csv");
        for (int i=0;i<nb_of_frame;i++)
        {
            myfile <<  Parameters[7*i+3]<<",";
            myfile << Parameters[7*i+4]<<",";
            myfile <<  Parameters[7*i+5]<<",";
            myfile << Parameters[7*i+6]<<",";
           
            myfile <<  Parameters[7*i]<<",";
            myfile << Parameters[7*i+1]<<",";
            myfile <<  Parameters[7*i+2]<<"\n";
            
        }

        myfile.close();

        std::ofstream myfile2;
        myfile2.open ("../data/data_opti/points.csv");
       

        for (int i=0;i<nb_of_landmark;i++){
            myfile2 <<  Parameters[7*nb_of_frame+3*i]<<",";
            myfile2 << Parameters[7*nb_of_frame+3*i+1]<<",";
            myfile2 <<  Parameters[7*nb_of_frame+3*i+2]<<"\n";

        }
        myfile2.close();
    
        std::ofstream myfile3;
        myfile3.open ("../data/data_opti/RMSE.csv");
       

        for (int i=0;i<nb_of_frame;i++){
            myfile3 <<i<<" , "<< RMSE_details[i]<<"\n";

        }
        myfile3.close();
        
    }
    return 0; 
}