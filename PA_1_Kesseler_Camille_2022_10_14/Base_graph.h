#include <eigen3/Eigen/Dense>
#include <list>
#include <iostream>

using namespace Eigen; 
using namespace std; 
//////////////////////Structure Pose Node ///////////////////////
struct Node_position_2D
{
  int index; 
  double x;
  double y;
  double theta; // angle in radian 
  int type ;// 0 for variable node 1 for fixe node
};

//////////////////////Structure Landmark Node ///////////////////////
struct Node_landmark_2D
{
  int index; 
  double x;
  double y;
  int type ;// 0 for variable node 1 for fixe node
};

//////////////////////Structure Edge ///////////////////////
struct Edge_2D
{
  int type_edge;   //0: odometry , 1: landmark

  //index of start and end nodes 
  int index_node_i; 
  int index_node_j; 

  //measurement data between the 2 nodes
  double x_measurement; 
  double y_measurement; 
  double theta_measurement;     //radian angle

  MatrixXd sigma;           // Covariance matrix 
};

//////////////////////Structure PoseGraph ///////////////////////
struct PoseGraph
{
  //Main members: list of Nodes and edges 

  list<Node_position_2D> NodesPosition;
  list<Node_landmark_2D> NodesLandmark;
  list<Edge_2D> Edges; 
   
  //Function to convert angle from degre to rad and inversely
  double DegToRad(double angle)
  {
    return angle*(M_PI/180);
  }
  double RadToDeg(double angle)
  {
    return angle*(180/M_PI);
  }

  //Function that Add a position node to the graph 
  void AddNode(int index,double x,double y,double theta,int type)
  { 
    Node_position_2D node; 
    node.index=index;
    node.x=x;
    node.y=y;
    node.theta=DegToRad(theta);
    node.type=type; 
    NodesPosition.push_back(node); 

  }

  //Function that Add a landmark node to the graph 
  void AddNode(int index,double x,double y,int type)
  { 
    Node_landmark_2D node; 
    node.index=index;
    node.x=x;
    node.y=y;
    node.type=type; 
    NodesLandmark.push_back(node); 

  }

  //Function that Add an edge between two pose nodes to the graph 
  void AddEdge(int index_i,int index_j,Matrix3d sigma,double x,double y,double theta)
  { 
    Edge_2D edge; 

    edge.type_edge=0;
    edge.index_node_i=index_i;
    edge.index_node_j=index_j;
    edge.sigma=sigma;
    edge.theta_measurement=DegToRad(theta);
    edge.x_measurement=x;
    edge.y_measurement=y; 
    Edges.push_back(edge); 

  }

  //Function that Add an edge between a pose node and a landmark to the graph 
  void AddEdge(int index_i,int index_l,Matrix2d sigma,double depth,double theta)
  { 
    Edge_2D edge; 
    edge.type_edge=1; 

    edge.index_node_i=index_i;
    edge.index_node_j=index_l;
    edge.sigma=sigma;
    edge.theta_measurement=DegToRad(theta);
    edge.x_measurement=depth; 
    Edges.push_back(edge); 
  }

  //Function that Add an edge between two pose nodes detected at the same position in the graph
  //Loop closure edge
  void AddLoopClosure(int index_n,int index_o, Matrix3d sigma)
  {
    Edge_2D edge;
    edge.type_edge=0; 

    edge.index_node_i=index_n;
    edge.index_node_j=index_o;
    edge.sigma=sigma;

    edge.theta_measurement=DegToRad(-360);
    edge.x_measurement=0; 
    edge.y_measurement=0;
    Edges.push_back(edge); 
  }

  //Function that search through the existing pose nodes in the graph to find the one at a certain index  
  Node_position_2D GetNodePosition(int index_search)
  {
    //search through the graph for the node of index 
    list<Node_position_2D>::iterator it;
    for (it=NodesPosition.begin(); it!= NodesPosition.end();++it)
      {
        if (it->index==index_search)
          return *it; 
      }
    
    cout << "no node position found with this index"<<endl;
    Node_position_2D empty_node; 
    return empty_node; 
  }

 
  //Function that search through the existing landmark nodes in the graph to find the one at a certain index  
  Node_landmark_2D GetNodeLandmark(int index_search)
  {
    //search through the graph for the node of index 
    list<Node_landmark_2D>::iterator it;
    for (it=NodesLandmark.begin(); it!= NodesLandmark.end();++it)
      {
        if (it->index==index_search)
          return *it; 
      }
    
    cout << "no node landmark found with this index"<<endl;
    Node_landmark_2D empty_node; 
    return empty_node; 
  }

  //Function that compute the error submatrix for an edge
  VectorXd error_ij(Edge_2D edge)  
  {
    VectorXd result;

    if (edge.type_edge==0)     //if the edge is between 2 pose nodes 
    {
      Node_position_2D node_i=GetNodePosition(edge.index_node_i);
      Node_position_2D node_j=GetNodePosition(edge.index_node_j);

      double ex= (node_j.x-node_i.x)*cos(edge.theta_measurement+node_i.theta)+(node_j.y-node_i.y)*sin(edge.theta_measurement+node_i.theta)-edge.x_measurement*cos(edge.theta_measurement)-edge.y_measurement*sin(edge.theta_measurement); 
      double ey=-(node_j.x-node_i.x)*sin(edge.theta_measurement+node_i.theta)+(node_j.y-node_i.y)*cos(edge.theta_measurement+node_i.theta)+edge.x_measurement*sin(edge.theta_measurement)-edge.y_measurement*cos(edge.theta_measurement);
      double eT= node_j.theta-node_i.theta-edge.theta_measurement; 

      result= Vector3d(ex,ey,eT);
    }
    else if(edge.type_edge==1)       //if the edge is between a pose node and a landmark
    {
      Node_position_2D node_i=GetNodePosition(edge.index_node_i);
      Node_landmark_2D node_l=GetNodeLandmark(edge.index_node_j);

      double ed=sqrt((node_l.x-node_i.x)*(node_l.x-node_i.x)+(node_l.y-node_i.y)*(node_l.y-node_i.y))-edge.x_measurement; 
      double eT= atan2((node_l.y-node_i.y),(node_l.x-node_i.x))-node_i.theta-edge.theta_measurement;
      
      result= Vector2d(ed,eT);

    }
    
    return  result; 
    
  }

  //Function that compute the Aij submatrix for an edge 
  MatrixXd A_ij(Edge_2D edge)  
  {
    MatrixXd result; 

    if(edge.type_edge==0)     //if the edge is between 2 pose nodes 
    {

      Node_position_2D node_i=GetNodePosition(edge.index_node_i);
      Node_position_2D node_j=GetNodePosition(edge.index_node_j);

      result=MatrixXd::Zero(3,3);

      result(0,0)=-cos(edge.theta_measurement+node_i.theta); 
      result(1,0)=sin(edge.theta_measurement+node_i.theta); 
      result(2,0)=0;
      result(0,1)=-sin(edge.theta_measurement+node_i.theta); 
      result(1,1)=-cos(edge.theta_measurement+node_i.theta); 
      result(2,1)=0;
      result(0,2)=-(node_j.x-node_i.x)*sin(edge.theta_measurement+node_i.theta)+(node_j.y-node_i.y)*cos(edge.theta_measurement+node_i.theta); 
      result(1,2)=-(node_j.x-node_i.x)*cos(edge.theta_measurement+node_i.theta)-(node_j.y-node_i.y)*sin(edge.theta_measurement+node_i.theta); 
      result(2,2)=-1;
    }
    else if(edge.type_edge==1)           //if the edge is between a pose node and a landmark
    {
      Node_position_2D node_i=GetNodePosition(edge.index_node_i);
      Node_landmark_2D node_l=GetNodeLandmark(edge.index_node_j);

      result=MatrixXd::Zero(2,3);

      result(0,0)=-(node_l.x-node_i.x)/sqrt((node_i.x-node_l.x)*(node_i.x-node_l.x)+(node_i.y-node_l.y)*(node_i.y-node_l.y)); 
      result(1,0)=(node_l.y-node_i.y)/((node_i.x-node_l.x)*(node_i.x-node_l.x)+(node_i.y-node_l.y)*(node_i.y-node_l.y)); 
      result(0,1)=-(node_l.y-node_i.y)/sqrt((node_i.x-node_l.x)*(node_i.x-node_l.x)+(node_i.y-node_l.y)*(node_i.y-node_l.y)); 
      result(1,1)=-(node_l.x-node_i.x)/((node_i.x-node_l.x)*(node_i.x-node_l.x)+(node_i.y-node_l.y)*(node_i.y-node_l.y)); 
      result(0,2)=0; 
      result(1,2)=-1; 
    
    }

    return result; 

  }

  //Function that compute the Bij submatrix for an edge 
  MatrixXd B_ij(Edge_2D edge)
  {

     MatrixXd result; 

    if(edge.type_edge==0)     //if the edge is between 2 pose nodes 
    {

      Node_position_2D node_i=GetNodePosition(edge.index_node_i);
      
      result=MatrixXd::Zero(3,3);

      result(0,0)=cos(edge.theta_measurement+node_i.theta); 
      result(1,0)=-sin(edge.theta_measurement+node_i.theta); 
      result(2,0)=0;
      result(0,1)=sin(edge.theta_measurement+node_i.theta); 
      result(1,1)=cos(edge.theta_measurement+node_i.theta); 
      result(2,1)=0;
      result(0,2)=0; 
      result(1,2)=0; 
      result(2,2)=1;
    }
    else if(edge.type_edge==1)        //if the edge is between a pose node and a landmark
    {
      Node_position_2D node_i=GetNodePosition(edge.index_node_i);
      Node_landmark_2D node_l=GetNodeLandmark(edge.index_node_j);
      
      result=MatrixXd::Zero(2,2);

      result(0,0)=(node_l.x-node_i.x)/sqrt((node_i.x-node_l.x)*(node_i.x-node_l.x)+(node_i.y-node_l.y)*(node_i.y-node_l.y)); 
      result(1,0)=-(node_l.y-node_i.y)/((node_i.x-node_l.x)*(node_i.x-node_l.x)+(node_i.y-node_l.y)*(node_i.y-node_l.y)); 
      result(0,1)=(node_l.y-node_i.y)/sqrt((node_i.x-node_l.x)*(node_i.x-node_l.x)+(node_i.y-node_l.y)*(node_i.y-node_l.y)); 
      result(1,1)=(node_l.x-node_i.x)/((node_i.x-node_l.x)*(node_i.x-node_l.x)+(node_i.y-node_l.y)*(node_i.y-node_l.y));

    }
    
    return result; 
  }
  
  //Function that optimize the node information using the edge measurement 
  int optimize(double convSeuil,int nIteration) 
  {

    cout<<"Start the optimization"<<endl; 
    
    double error_Tot=10000;
    int iteration=0;  

    while (error_Tot>convSeuil && iteration<nIteration)    //keep try until we have done nIteration or we do not change the results
    {
      cout<<"We are doing iteration "<<iteration<<endl;

      //Initialize the matrix H and b as empty 
      int matrix_dim_position=NodesPosition.size();
      int matrix_dim_landmark=NodesLandmark.size(); 

      MatrixXd H(matrix_dim_position*3+matrix_dim_landmark*2,matrix_dim_position*3+matrix_dim_landmark*2);
      H=MatrixXd::Zero(matrix_dim_position*3+matrix_dim_landmark*2,matrix_dim_position*3+matrix_dim_landmark*2);

      VectorXd b(matrix_dim_position*3+matrix_dim_landmark*2);
      b=VectorXd::Zero(matrix_dim_position*3+matrix_dim_landmark*2); 
 
      //for all edges fill the matrix H and b
      list<Edge_2D>::iterator it;
      for (it=Edges.begin(); it!= Edges.end();++it)
      {

        int index_i=it->index_node_i; 
        int index_j=it->index_node_j; 

        if (it->type_edge==0)         ///if the edge is between 2 pose nodes 
        {
          //create matrix Aij
          Matrix3d Aij= A_ij(*it); 

          //create matrix Bij
          Matrix3d Bij= B_ij(*it); 

          //compute error ije
          Vector3d eij= error_ij(*it); 

          //filling the matrix H and b
          H.block<3,3>(3*index_i,3*index_i)+=Aij.transpose()*it->sigma *Aij;
          H.block<3,3>(3*index_i,3*index_j)+=Aij.transpose()*it->sigma *Bij;
          H.block<3,3>(3*index_j,3*index_i)+=Bij.transpose()*it->sigma *Aij;
          H.block<3,3>(3*index_j,3*index_j)+=Bij.transpose()*it->sigma *Bij;
          b.segment(3*index_i,3)+=eij.transpose()*it->sigma*Aij;
          b.segment(3*index_j,3)+=eij.transpose()*it->sigma*Bij; 
        }
        else if (it->type_edge==1)         //if the edge is between a pose node and a landmark
        {
          //create matrix Aij
          MatrixXd Aij(2,3);
          Aij= A_ij(*it); 

          //create matrix Bij
          Matrix2d Bij= B_ij(*it); 

          //compute error ije
          Vector2d eij= error_ij(*it); 
          
          //filling the matrix H and b
          H.block<3,3>(3*index_i,3*index_i)+=Aij.transpose()*it->sigma *Aij;
          H.block<3,2>(3*index_i,3*matrix_dim_position+2*index_j)+=Aij.transpose()*it->sigma *Bij;
          H.block<2,3>(3*matrix_dim_position+2*index_j,3*index_i)+=Bij.transpose()*it->sigma *Aij;
          H.block<2,2>(3*matrix_dim_position+2*index_j,3*matrix_dim_position+2*index_j)+=Bij.transpose()*it->sigma *Bij;
          b.segment(3*index_i,3)+=eij.transpose()*it->sigma*Aij;
          b.segment(3*matrix_dim_position+2*index_j,2)+=eij.transpose()*it->sigma*Bij; 
        }
      }
      
      // Fixe position node so we add the identity to inverse H  
      list<Node_position_2D>::iterator it2;
      for (it2=NodesPosition.begin(); it2!= NodesPosition.end();++it2)
      {
        if (it2->type==1)
        {
          H.block<3,3>(3*it2->index,3*it2->index)+=MatrixXd::Identity(3,3);
        }
      }

      // solve the linear system H*x=-b with Eigen 
      VectorXd x;
      x=H.ldlt().solve(-1*b);

      //Update  the pose node  in the graph 
      list<Node_position_2D>::iterator it3;
      for (it3=NodesPosition.begin(); it3!= NodesPosition.end();++it3)
      {
        if (it3->type==0)
        {
          it3->x+=x(3*it3->index);
          it3->y+=x(3*it3->index+1);
          it3->theta+=x(3*it3->index+2);
        }
      }


      //Update  the landmark node in the graph 
      list<Node_landmark_2D>::iterator it4;
      for (it4=NodesLandmark.begin(); it4!= NodesLandmark.end();++it4)
      {

          it4->x+=x(3*matrix_dim_position+ 2*it4->index);
          it4->y+=x(3*matrix_dim_position+2*it4->index+1);

      }

      plot_nodes();
      //Update the criterion for iterating optimization 
      error_Tot=0;
      for (int i=0;i<x.size();i++)
      {
        error_Tot+=x(i)*x(i); 
      }
      error_Tot=sqrt(error_Tot);
      iteration+=1;

      cout<<"the error_Tot is "<<error_Tot<<endl; 

    }

    return 0; 
  }

  //Function that print the node information in the console 
  int plot_nodes()
  {
    list<Node_position_2D>::iterator it;
    for (it=NodesPosition.begin(); it!= NodesPosition.end();++it)
      {
        cout<<" Position node "<<it->index <<"  x: "<<it->x<<"  y: "<<it->y<<"  theta: "<<RadToDeg(it->theta)<<endl;
      }

    list<Node_landmark_2D>::iterator it2;
    for (it2=NodesLandmark.begin(); it2!= NodesLandmark.end();++it2)
      {
        cout<<" landmark node "<<it2->index <<"  x: "<<it2->x<<"  y: "<<it2->y<<endl;
      }
    return 0; 
  }
};


