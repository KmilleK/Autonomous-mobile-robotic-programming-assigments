#include <eigen3/Eigen/Dense>
#include <list>
#include <iostream>
#include <fstream>

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
   
  //Function that Add a position node to the graph 
  void AddNode(int index,double x,double y,double theta,int type)
  { 
    //Check if the node is already existing  
    bool new_node=true; 

    list<Node_position_2D>::iterator it;
    for (it=NodesPosition.begin(); it!= NodesPosition.end();++it)
      {
        if (it->index==index)
          {
            it->x=x;
            it->y=y;
            it->theta=theta; 
            it->type=type;
            new_node=false;
          }
          
      }
    
    if (new_node)
    {
      Node_position_2D node; 
      node.index=index;
      node.x=x;
      node.y=y;
      node.theta=theta;
      node.type=type; 
      NodesPosition.push_back(node); 
    }

  }

  void AddNode(int index,double x,double y,int type)
  { 

     //Check if existing node 
    bool new_node=true; 

    list<Node_landmark_2D>::iterator it;
    for (it=NodesLandmark.begin(); it!= NodesLandmark.end();++it)
      {
        if (it->index==index)
          {
            it->x=x;
            it->y=y;
            it->type=type;
            new_node=false;
          }
          
      }

    if (new_node)
    {
      Node_landmark_2D node; 
      node.index=index;
      node.x=x;
      node.y=y;
      node.type=type; 
      NodesLandmark.push_back(node); 
    }
  }


  void AddEdge(int index_i,int index_j,Matrix3d sigma,double x,double y,double theta)
  { 
      //Check if existing node index_i , index_j
    bool new_node_i=true;
    bool new_node_j=true; 

    list<Node_position_2D>::iterator it;
    for (it=NodesPosition.begin(); it!= NodesPosition.end();++it)
      {
        if (it->index==index_i)
          {
            new_node_i=false;
          }
        
        if (it->index==index_j)
          {
            new_node_j=false;
          }
      }
    
    if (new_node_i)
    {
      Node_position_2D node; 
      node.index=index_i;
      NodesPosition.push_back(node); 
    }
    if (new_node_j)
    {
      Node_position_2D node; 
      node.index=index_j;
      NodesPosition.push_back(node); 
    }
    
    
    //Add edge 

    Edge_2D edge; 

    edge.type_edge=0;
    edge.index_node_i=index_i;
    edge.index_node_j=index_j;
    edge.sigma=sigma;
    edge.theta_measurement=theta;
    edge.x_measurement=x;
    edge.y_measurement=y; 
    Edges.push_back(edge); 

  }
  
  void AddEdge(int index_i,int index_l,Matrix2d sigma,double x,double y)
  { 
    //Check if existing node index_i , index_j
    bool new_node_i=true;
    bool new_node_l=true; 

    list<Node_position_2D>::iterator it;
    for (it=NodesPosition.begin(); it!= NodesPosition.end();++it)
      {
        if (it->index==index_i)
          {
            new_node_i=false;
          }
           
      }
    
    list<Node_landmark_2D>::iterator it2;
    for (it2=NodesLandmark.begin(); it2!= NodesLandmark.end();++it2)
      {
        if (it2->index==index_l)
          {
            new_node_l=false;
          }
           
      }

    
    
    if (new_node_i)
    {
      Node_position_2D node; 
      node.index=index_i;
      NodesPosition.push_back(node); 
    }
    if (new_node_l)
    {
      Node_landmark_2D node; 
      node.index=index_l;
      NodesLandmark.push_back(node); 
    }

    Edge_2D edge; 
    edge.type_edge=1; 

    edge.index_node_i=index_i;
    edge.index_node_j=index_l;
    edge.sigma=sigma;
    edge.y_measurement=y;
    edge.x_measurement=x; 
    Edges.push_back(edge); 
  }

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

  VectorXd error_ij(Edge_2D edge)  //compute the error for an edges 
  {
    VectorXd result;
    if (edge.type_edge==0)
    {
      Node_position_2D node_i=GetNodePosition(edge.index_node_i);
      Node_position_2D node_j=GetNodePosition(edge.index_node_j);


      double ex= (node_j.x-node_i.x)*cos(edge.theta_measurement+node_i.theta)+(node_j.y-node_i.y)*sin(edge.theta_measurement+node_i.theta)-edge.x_measurement*cos(edge.theta_measurement)-edge.y_measurement*sin(edge.theta_measurement); 
      double ey=-(node_j.x-node_i.x)*sin(edge.theta_measurement+node_i.theta)+(node_j.y-node_i.y)*cos(edge.theta_measurement+node_i.theta)+edge.x_measurement*sin(edge.theta_measurement)-edge.y_measurement*cos(edge.theta_measurement);
      double eT= node_j.theta-node_i.theta-edge.theta_measurement; 

      result= Vector3d(ex,ey,eT);
    }
    else if(edge.type_edge==1)
    {
      Node_landmark_2D node_l=GetNodeLandmark(edge.index_node_j);
      double ed=node_l.x-edge.x_measurement; 
      double eT= node_l.y-edge.y_measurement;
      result= Vector2d(ed,eT);

    }
    
    return  result; 
    
  }

  MatrixXd A_ij(Edge_2D edge)  //compute the A submatrix
  {
    MatrixXd result; 

    if(edge.type_edge==0)
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
    else if(edge.type_edge==1)
    {
     
      result=MatrixXd::Zero(2,3);

         
    }

    return result; 

  }

  MatrixXd B_ij(Edge_2D edge) // compute the B submatrix 
  {

     MatrixXd result; 

    if(edge.type_edge==0)
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
    else if(edge.type_edge==1)
    {
      Node_position_2D node_i=GetNodePosition(edge.index_node_i);
      Node_landmark_2D node_l=GetNodeLandmark(edge.index_node_j);
      
      result=MatrixXd::Identity(2,2);

      
    }
    
    return result; 
  }
  

  int optimize(double convSeuil,int nIteration) // optimize 
  {

    cout<<"Start the optimization"<<endl; 
    double error_Tot=10000;
    int iteration=0;  
    //while (error_Tot>convSeuil && iteration<nIteration)
    //{
     // cout<<"we are doing iteration "<<iteration<<endl;

      //Initialize the matrix H and b, empty 
      int matrix_dim_position=NodesPosition.size();
      int matrix_dim_landmark=NodesLandmark.size(); 

      MatrixXd H(matrix_dim_position*3+matrix_dim_landmark*2,matrix_dim_position*3+matrix_dim_landmark*2);
      H=MatrixXd::Zero(matrix_dim_position*3+matrix_dim_landmark*2,matrix_dim_position*3+matrix_dim_landmark*2);

      VectorXd b(matrix_dim_position*3+matrix_dim_landmark*2);
      b=VectorXd::Zero(matrix_dim_position*3+matrix_dim_landmark*2); 

      //for all edges fill the matrix 
      list<Edge_2D>::iterator it;

      for (it=Edges.begin(); it!= Edges.end();++it)
      {
        // it is the edges
        int index_i=it->index_node_i; 
        int index_j=it->index_node_j; 
             
        if (it->type_edge==0)
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
        else if (it->type_edge==1)
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
      
      // fixe position node so we add the identity 
      list<Node_position_2D>::iterator it2;
      for (it2=NodesPosition.begin(); it2!= NodesPosition.end();++it2)
      {
        if (it2->type==1)
        {
          H.block<3,3>(3*it2->index,3*it2->index)+=MatrixXd::Identity(3,3);
        }
      }

      cout<<H.rows()<<endl;
      // solve the linear system H*x=-b
      cout<<"start solving "<<iteration<<endl;
      VectorXd x=VectorXd::Zero(matrix_dim_position*3+matrix_dim_landmark*2); ;
      x=H.ldlt().solve(-1*b);

      //change the node position in the graph 
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

      list<Node_landmark_2D>::iterator it4;
      for (it4=NodesLandmark.begin(); it4!= NodesLandmark.end();++it4)
      {
          it4->x+=x(3*matrix_dim_position+ 2*it4->index);
          it4->y+=x(3*matrix_dim_position+2*it4->index+1);
      }
       cout<<"we are finish iteration "<<iteration<<endl;
  /*
      //plot_nodes();
      //error_Tot=0;
      for (int i=0;i<x.size();i++)
      {
        error_Tot+=x(i)*x(i); 
      }
      error_Tot=sqrt(error_Tot);
      iteration+=1;

    }
*/
  

    return 0; 
  }
    int save_nodes()
    {
      string output_nodes="../VictoriaPark_Data/My_result_VP.txt";
      ofstream myfile (output_nodes);
       
      if(myfile.is_open())  
      {
        list<Node_position_2D>::iterator it;
        for (it=NodesPosition.begin(); it!= NodesPosition.end();++it)
          {
            myfile <<" "<<it->x<<" "<<it->y<<" "<<(it->theta)<<" \n";
          }
        myfile.close();
      }
      else cout<<"cannot open the save file for node"<<endl;

      return 0; 
    }

    int save_landmark()
    {
      
      string output_nodes="../VictoriaPark_Data/My_result_VP_lm.txt";
      ofstream myfile (output_nodes);
       
      if(myfile.is_open())  
      {
        list<Node_landmark_2D>::iterator it2;
        for (it2=NodesLandmark.begin(); it2!= NodesLandmark.end();++it2)
          {
            myfile <<it2->x<<" "<<it2->y<<" \n";
          }
        myfile.close();
      }
      else cout<<"cannot open the save file for node"<<endl;


      return 0; 
    }
  
};


