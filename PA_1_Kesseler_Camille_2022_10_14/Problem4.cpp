#include "Base_graph.h"

int main()
{

    // Graph structure creation 
    PoseGraph SLAM_graph; 

    //Pose node data addition 
    SLAM_graph.AddNode(0,0,0,0,1);   //fixe point 0 
    SLAM_graph.AddNode(1,19,9.5,88,0);  
    SLAM_graph.AddNode(2,19,18,92,0); 
    SLAM_graph.AddNode(3,0,21,170,0); 
    SLAM_graph.AddNode(4,5,5,0,0);   

    //Data node landmark        
    SLAM_graph.AddNode(0,7,24,0);
    SLAM_graph.AddNode(1,15,29,0);

    //Data edge 
    Matrix3d sigma3= MatrixXd::Identity(3,3); 
    SLAM_graph.AddEdge(0,1,sigma3,20,10,90);
    SLAM_graph.AddEdge(1,2,sigma3,10,0,0);
    SLAM_graph.AddEdge(2,3,sigma3,0,20,90);
    SLAM_graph.AddEdge(3,4,sigma3,0,19,175); //new edge for problem 2
   

    //edge for landmark
    Matrix2d sigma2=MatrixXd::Identity(2,2);
    SLAM_graph.AddEdge(0,0,sigma2,26,70);  
    SLAM_graph.AddEdge(0,1,sigma2,31,60);
    SLAM_graph.AddEdge(1,0,sigma2,19,35);
    SLAM_graph.AddEdge(1,1,sigma2,19,15);
    SLAM_graph.AddEdge(2,0,sigma2,13,70);
    SLAM_graph.AddEdge(2,1,sigma2,10,40);

    //loop closure edge 
    SLAM_graph.AddLoopClosure(4,0,sigma3); 

    SLAM_graph.plot_nodes();
   
    //optimize the data   
    double conv_seuil=0.001; 
    int nb_max_iteration=10; 
    SLAM_graph.optimize(conv_seuil,nb_max_iteration); 

    return 0;

}