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
    SLAM_graph.AddNode(4,5,5,0,0);   //new point for problem 2 
    
    //Edges data addition 
    Matrix3d sigma= MatrixXd::Identity(3,3); 
    SLAM_graph.AddEdge(0,1,sigma,20,10,90);
    SLAM_graph.AddEdge(1,2,sigma,10,0,0);
    SLAM_graph.AddEdge(2,3,sigma,0,20,90);
    SLAM_graph.AddEdge(3,4,sigma,0,19,175); //new edge for problem 2
    
    //loop closure edge addition 
    SLAM_graph.AddLoopClosure(4,0,sigma); 

    SLAM_graph.plot_nodes();   //plot initial data
    
    //optimize the data 
    double conv_seuil=0.001; 
    int nb_max_iteration=10; 
    SLAM_graph.optimize(conv_seuil,nb_max_iteration); 

    return 0;

}