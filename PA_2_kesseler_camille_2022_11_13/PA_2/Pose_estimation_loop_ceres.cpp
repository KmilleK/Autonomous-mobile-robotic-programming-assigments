/***
 *  Autonomous Mobile robotics, fall 2022,
 *  Programming assignment 2 
 * 
 *  Author: Kesseler Camille 
 * 
 *  Pose estimation: Basic bundle adjustement for pose and landmark position estimation. 
 *  Here we use Ceres to solve the problem.
 * 
 ***/

// history: 
// 3/11/2022: creation file, ceres problem definition

//to do: function definition / data insertion 

#include "ceres/ceres.h"

// Function that define the reprojection error for pose/pose edge 
struct ErrorFunctionPosePose {
    ErrorFunctionPosePose(double observed_x, double observed_y,double observed_theta)
        : observed_x(observed_x), observed_y(observed_y),observed_theta(observed_theta) {}   

    template <typename T>
    bool operator()(const T* const pose_i,
                  const T* const pose_j,
                  T* residuals) const {

    //Cos and Sin computation as it is in radian

    T sin_ij=sin(T(observed_theta)*M_PI/ T(180));
    T cos_ij=cos(T(observed_theta)*M_PI/ T(180));
    T sin_i=sin((pose_i[2]+T(observed_theta))*M_PI/ T(180));
    T cos_i=cos((pose_i[2]+T(observed_theta))*M_PI/ T(180));

    // The error is the difference between the predicted and observed position

    residuals[0] = (pose_j[0]-pose_i[0])*cos_i+ (pose_j[1]-pose_i[1])*sin_i - T(observed_x)*cos_ij- T(observed_y)*sin_ij;
    residuals[1] = -(pose_j[0]-pose_i[0])*sin_i+ (pose_j[1]-pose_i[1])*cos_i + T(observed_x)*sin_ij- T(observed_y)*cos_ij;
    residuals[2] = pose_j[2]-pose_i[2]-T(observed_theta); 
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y,
                                      const double observed_theta) {
     return (new ceres::AutoDiffCostFunction<ErrorFunctionPosePose, 3,3,3>(
                 new ErrorFunctionPosePose(observed_x, observed_y, observed_theta)));
   }

  // the observation data stored when create the structure 
  double observed_x;
  double observed_y;
  double observed_theta;
};

// Function that define the reprojection error for landmark/pose edge

struct ErrorFunctionPoseLandmark {
    ErrorFunctionPoseLandmark(double observed_d, double observed_theta)
        : observed_d(observed_d), observed_theta(observed_theta) {}   

    template <typename T>
    bool operator()(const T* const pose_i,
                  const T* const landmark_j,
                  T* residuals) const {

    // The error is the difference between the predicted and observed position

    residuals[0] = sqrt((landmark_j[0]-pose_i[0])*(landmark_j[0]-pose_i[0])+(landmark_j[1]-pose_i[1])*(landmark_j[1]-pose_i[1])) - T(observed_d);
    residuals[1] = atan2((landmark_j[1]-pose_i[1]),(landmark_j[0]-pose_i[0]))*T(180)/M_PI-pose_i[2] -T(observed_theta);
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double observed_d,                                    
                                      const double observed_theta) {
     return (new ceres::AutoDiffCostFunction<ErrorFunctionPoseLandmark,2,3,2>(
                 new ErrorFunctionPoseLandmark(observed_d, observed_theta)));
   }

  // the observation data stored when create the structure 
  double observed_d;
  double observed_theta;
};


int main()
{


    //Initial Data creation

    double Pose_0[3]={0,0,0};
    double Pose_1[3]={19,9.5,88};
    double Pose_2[3]={19,18,92};
    double Pose_3[3]={0,21,170};
    double Pose_4[3]={5,5,0};
    
    double Landmark_0[2]={7,24};
    double Landmark_1[2]={15,29};
   
    
    // Ceres problem creation

    ceres::Problem Pose_graph; 

    // problem filling with information

    //odometry 0-1
    ceres::CostFunction* cost_function01 = ErrorFunctionPosePose::Create(20,10,90);
    Pose_graph.AddResidualBlock(cost_function01,                     //error function 
                                nullptr /* squared loss */,             //loss_function to reduce outlier influence (here f(x)=x)
                                Pose_0, Pose_1);                                      // initial position definition

    //odometry 1-2
    ceres::CostFunction* cost_function12 = ErrorFunctionPosePose::Create(10,0,0);
    Pose_graph.AddResidualBlock(cost_function12,                     //error function 
                                nullptr /* squared loss */,             //loss_function to reduce outlier influence (here f(x)=x)
                                Pose_1, Pose_2);                                      // initial position definition

    //odometry 2-3
    ceres::CostFunction* cost_function23 = ErrorFunctionPosePose::Create(0,20,90);
    Pose_graph.AddResidualBlock(cost_function23,                     //error function 
                                nullptr /* squared loss */,             //loss_function to reduce outlier influence (here f(x)=x)
                                Pose_2, Pose_3);                                      // initial position definition

    //odometry 3-4
    ceres::CostFunction* cost_function34 = ErrorFunctionPosePose::Create(0,19,175);
    Pose_graph.AddResidualBlock(cost_function34,                     //error function 
                                nullptr /* squared loss */,             //loss_function to reduce outlier influence (here f(x)=x)
                                Pose_3, Pose_4);                                      // initial position definition

    //loop closure between 2 node 0 and 4

    ceres::CostFunction* cost_function40 = ErrorFunctionPosePose::Create(0,0,360);
    Pose_graph.AddResidualBlock(cost_function40,                     //error function 
                                nullptr /* squared loss */,             //loss_function to reduce outlier influence (here f(x)=x)
                                Pose_0, Pose_4); 

    //measurement 0-0

    ceres::CostFunction* measurement_function_00 = ErrorFunctionPoseLandmark::Create(26,70);
    Pose_graph.AddResidualBlock(measurement_function_00,                     //error function 
                                nullptr /* squared loss */,             
                                Pose_0, Landmark_0);                                      

    //measurement 0-1
    ceres::CostFunction* measurement_function_01 = ErrorFunctionPoseLandmark::Create(31,60);
    Pose_graph.AddResidualBlock(measurement_function_01,                     //error function 
                                nullptr /* squared loss */,             
                                Pose_0, Landmark_1);

    //measurement 1-0
    ceres::CostFunction* measurement_function_10 = ErrorFunctionPoseLandmark::Create(19,35);
    Pose_graph.AddResidualBlock(measurement_function_10,                     //error function 
                                nullptr /* squared loss */,             
                                Pose_1, Landmark_0);
    //measurement 1-1
    ceres::CostFunction* measurement_function_11 = ErrorFunctionPoseLandmark::Create(19,15);
    Pose_graph.AddResidualBlock(measurement_function_11,                     //error function 
                                nullptr /* squared loss */,             
                                Pose_1, Landmark_1);
    //measurement 2-0
    ceres::CostFunction* measurement_function_20 = ErrorFunctionPoseLandmark::Create(13,70);
    Pose_graph.AddResidualBlock(measurement_function_20,                     //error function 
                                nullptr /* squared loss */,             
                                Pose_2, Landmark_0);
    //measurement 2-1
    ceres::CostFunction* measurement_function_21 = ErrorFunctionPoseLandmark::Create(10,40);
    Pose_graph.AddResidualBlock(measurement_function_21,                     //error function 
                                nullptr /* squared loss */,             
                                Pose_2, Landmark_1);

    // Need to fix first node as we have too much freedom ( whitout it resolve but not at good position)

    Pose_graph.SetParameterBlockConstant(Pose_0); 

    // Ceres solver option definition 

    ceres::Solver::Options solver_options;
    //solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    solver_options.linear_solver_type = ceres::DENSE_SCHUR;
    solver_options.minimizer_progress_to_stdout = true;
    
    

    // Solving the problem and ouput the report 

    ceres::Solver::Summary Graph_solve_summary;
    ceres::Solve(solver_options, &Pose_graph, &Graph_solve_summary);
    std::cout << Graph_solve_summary.FullReport() << "\n";

    std::cout <<"Pose 0 x: "<< Pose_0[0]<<" y: "<< Pose_0[1]<< " theta: "<<Pose_0[2]<<std::endl;
    std::cout <<"Pose 1 x: "<< Pose_1[0]<<" y: "<< Pose_1[1]<< " theta: "<<Pose_1[2]<<std::endl;
    std::cout <<"Pose 2 x: "<< Pose_2[0]<<" y: "<< Pose_2[1]<< " theta: "<<Pose_2[2]<<std::endl;
    std::cout <<"Pose 3 x: "<< Pose_3[0]<<" y: "<< Pose_3[1]<< " theta: "<<Pose_3[2]<<std::endl;
    std::cout <<"Pose 4 x: "<< Pose_4[0]<<" y: "<< Pose_4[1]<< " theta: "<<Pose_4[2]<<std::endl;

    std::cout <<"Landmark 1 x: "<< Landmark_0[0]<<" y: "<< Landmark_0[1]<< std::endl;
    std::cout <<"Landmark 2 x: "<< Landmark_1[0]<<" y: "<< Landmark_1[1]<< std::endl;
    
    return 0;
}