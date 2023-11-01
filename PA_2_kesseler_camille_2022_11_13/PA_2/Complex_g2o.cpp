#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "se2.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>

using namespace g2o; 
using namespace std; 
using namespace Eigen; 


class VertexSE2 : public BaseVertex<3, SE2>{

 public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
virtual void setToOriginImpl() { _estimate = SE2(); }

  virtual void oplusImpl(const double* update) {
    SE2 up(update[0], update[1], update[2]);
    _estimate *= up;
  }


bool read(std::istream& is) {
  Eigen::Vector3d p;
  is >> p[0] >> p[1] >> p[2];
  _estimate.fromVector(p);
  return true;
}

bool write(std::ostream& os) const {
  Eigen::Vector3d p = estimate().toVector();
  os << p[0] << " " << p[1] << " " << p[2];
  return os.good();
}

};

class VertexPointXY : public BaseVertex<2, Vector2d> {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() { _estimate.setZero(); }

  virtual void oplusImpl(const double* update) {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
  }

 
bool read(std::istream& is) {
  is >> _estimate[0] >> _estimate[1];
  return true;
}

bool write(std::ostream& os) const {
  os << estimate()(0) << " " << estimate()(1);
  return os.good();

}

};

 
class EdgeSE2 : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  virtual void computeError() override {
    const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
    const VertexSE2* v2 = static_cast<const VertexSE2*>(_vertices[1]);
    SE2 delta =_inverseMeasurement * (v1->estimate().inverse() * v2->estimate());
    _error = delta.toVector();
  }

  virtual void setMeasurement(const SE2& m) override {
    _measurement = m;
    _inverseMeasurement = m.inverse();
  }

  bool read(std::istream& is) {
  Vector3d p;
  is >> p[0] >> p[1] >> p[2];
  _measurement.fromVector(p);
  _inverseMeasurement = measurement().inverse();
  for (int i = 0; i < 3; ++i)
    for (int j = i; j < 3; ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}

    bool write(std::ostream& os) const {
  Vector3d p = measurement().toVector();
  os << p.x() << " " << p.y() << " " << p.z();
  for (int i = 0; i < 3; ++i)
    for (int j = i; j < 3; ++j) os << " " << information()(i, j);
  return os.good();
}

 protected:
  SE2 _inverseMeasurement;
};


class EdgeSE2PointXY : public BaseBinaryEdge<2,Eigen::Vector2d, VertexSE2, VertexPointXY>
{
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    void computeError()
    {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexPointXY* landmark = static_cast<const VertexPointXY*>(_vertices[1]);
        
        SE2 pose =v1->estimate(); 
        Vector2d l=landmark->estimate(); 
  
        //to do 
        _error =  pose.inverse()*l -_measurement; 
    }
    
   
    bool read(std::istream& is) {
        is >> _measurement[0] >> _measurement[1];
        is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
        information()(1, 0) = information()(0, 1);
        return true;
        }

    bool write(std::ostream& os) const {
        os << measurement()[0] << " " << measurement()[1] << " ";
        os << information()(0, 0) << " " << information()(0, 1) << " "
            << information()(1, 1);
        return os.good();
        }




};


void split_string(string input,char delim,vector<string> &output)
{
    stringstream ss(input);
    string s;
    while(getline(ss,s,delim))
    {
        //cout<<s<<endl;
        output.push_back(s);
    }

}


int main()
{
    //Input
    string input_data="../VictoriaPark_Data/mh_T2_victoriaPark_01.txt";
    string input_pos_ini="../VictoriaPark_Data/ISAM2_INIT_GUESS_victoriaPark.txt";
    string input_landmark_ini="../VictoriaPark_Data/ISAM2_INIT_GUESS_victoriaPark_lm.txt";
     
    // Creation of the g2o graph 

    g2o::SparseOptimizer* GraphOptimizer=  new g2o::SparseOptimizer; 

    // solver definition 
    typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;

    auto solver = g2o::make_unique<LinearSolverEigen<SlamBlockSolver::PoseMatrixType>>(); 

    // optimization algorithm: 
    g2o::OptimizationAlgorithmLevenberg* optiAlgo = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<SlamBlockSolver>(std::move(solver))); 
    // setup the optimizer 

    GraphOptimizer->setAlgorithm(optiAlgo); 

    // Initialize the position node 

    ifstream myfile2(input_pos_ini);
    string line2;
    int index = 0;
    if (myfile2.is_open())
    {
    while(getline(myfile2,line2))
    {   

        vector<string> data;
        split_string(line2,' ',data);

        double x=stod(data[0].c_str());
        double y=stod(data[1].c_str());
        double theta=stod(data[2].c_str());
        
        //Iterate though each line which is a node 
        const SE2& pose= SE2(x,y,theta); 
        VertexSE2* Nodepose = new VertexSE2; 
        Nodepose->setId(index);
        Nodepose->setEstimate(pose); 
        GraphOptimizer->addVertex(Nodepose); 

        index+=1;

    }
    }
    else cout<< "cannot open the file: "<<input_pos_ini<<endl;
    
    int Landmark_offset=index;

    // Initialize the landmark 
    ifstream myfile3(input_landmark_ini);
    string line3;
    if (myfile3.is_open())
    {          
    while(getline(myfile3,line3))
    {   
        //Iterate though each line to fill the pose graph 
        vector<string> data;
        split_string(line3,' ',data);

        double x=stod(data[0].c_str());
        double y=stod(data[1].c_str());
        
        //landmark 0
        VertexPointXY* landmark = new VertexPointXY;
        landmark->setId(index);
        landmark->setEstimate(Eigen::Vector2d(x,y));      // check if right 
        GraphOptimizer->addVertex(landmark);
    
        index+=1; 

    }
    }
    else cout<< "cannot open the file: "<<input_landmark_ini<<endl;
    
    // Fill pose graph with odometry
    ifstream myfile(input_data);
    string line;
    
    if (myfile.is_open())
    {    
        while(getline(myfile,line))
        {   
            //Iterate though each line to fill the pose graph 
            vector<string> data;
            split_string(line,' ',data);

            string type=data[0];
            if (data[0]=="ODOMETRY")
            {
                int index_i=stoi(data[1].c_str());
                int index_j=stoi(data[3].c_str());

                if (stoi(data[4].c_str())==0)       // no loop closure detect
                {   
                    //Add a normal odometry edge 
                    EdgeSE2 *Measurement = new EdgeSE2;
                    Measurement->vertices()[0]=GraphOptimizer->vertex(index_i);
                    Measurement->vertices()[1]=GraphOptimizer->vertex(index_j);
                    const SE2& measure= SE2(stod(data[6].c_str()),stod(data[7].c_str()),stod(data[8].c_str())); // how to define SE2
                    Measurement->setMeasurement(measure);    // how to define 
                    Measurement->setInformation(Eigen::Matrix3d::Identity());
                    GraphOptimizer->addEdge(Measurement); 
                   
                   
                }
                else if (stoi(data[4].c_str())==1)      //loop closure detection 
                {
                    EdgeSE2 *Measurement = new EdgeSE2;
                    Measurement->vertices()[0]=GraphOptimizer->vertex(index_i);
                    Measurement->vertices()[1]=GraphOptimizer->vertex(index_j);
                    const SE2& measure= SE2(stod(data[6].c_str()),stod(data[7].c_str()),stod(data[8].c_str())); // how to define SE2
                    Measurement->setMeasurement(measure);    // how to define 
                    Measurement->setInformation(Eigen::Matrix3d::Identity());
                    GraphOptimizer->addEdge(Measurement); 
                    cout<<"there is a loop closure"<<endl;    
                }
                else
                    cout<<"wrong data in loop closure"<<endl;
                
            }
            else if (data[0]=="LANDMARK")
            {
                int index_i=stoi(data[1].c_str());
                
                if (stoi(data[2].c_str())==1)     // no ambiguity
                {
                    //Add a normal landmark edge

                    int index_l=Landmark_offset+stoi(data[3].c_str());

                    EdgeSE2PointXY* landmarkObservation = new EdgeSE2PointXY;  // to change as the measurement model is different 
                    landmarkObservation->vertices()[0]=GraphOptimizer->vertex(index_i);
                    landmarkObservation->vertices()[1]=GraphOptimizer->vertex(index_l);
                    landmarkObservation->setMeasurement(Eigen::Vector2d(stod(data[6].c_str()),stod(data[7].c_str())));
                    landmarkObservation->setInformation(Eigen::Matrix2d::Identity());
                    GraphOptimizer->addEdge(landmarkObservation);
             
                }
                else if (stoi(data[2].c_str())==2)   //case of landmark ambiguity 
                {
                    int index_l=Landmark_offset+stoi(data[3].c_str());
                    
                    EdgeSE2PointXY* landmarkObservation = new EdgeSE2PointXY;  // to change as the measurement model is different 
                    landmarkObservation->vertices()[0]=GraphOptimizer->vertex(index_i);
                    landmarkObservation->vertices()[1]=GraphOptimizer->vertex(index_l);
                    landmarkObservation->setMeasurement(Eigen::Vector2d(stod(data[7].c_str()),stod(data[8].c_str())));
                    landmarkObservation->setInformation(Eigen::Matrix2d::Identity());
                    GraphOptimizer->addEdge(landmarkObservation);
             
                }
                else
                    cout<<"wrong data in number of landmark"<<endl;
                
            }
            else
                cout<<"wrong data in type"<<endl;
        
    
        }
    }
    else cout<< "cannot open the file: "<<input_data<<endl;
    

   
     //plot the initial data 
    

VertexSE2* InitialPoseRobot = dynamic_cast<VertexSE2*>(GraphOptimizer->vertex(0));
InitialPoseRobot->setFixed(true);

// optimization results: 

GraphOptimizer->setVerbose(true);  // print intermediate information

// initialize and run the optimizer
int nbIterations= 10; 

GraphOptimizer->initializeOptimization(); 
GraphOptimizer->optimize(nbIterations);

cout<<"optimization done"<<endl;

//Save the position node 
string output_nodes="../VictoriaPark_Data/My_result_VP.txt";
ofstream myfile_oo (output_nodes);
  
if(myfile_oo.is_open())  
{
  
  for (int it=0; it<Landmark_offset;++it)
    {
      const VertexSE2* NodePose = static_cast<const VertexSE2*>(GraphOptimizer->vertex(it));
      Vector3d Pose =NodePose->estimate().toVector();
      myfile_oo <<" "<<Pose[0]<<" "<<Pose[1]<<" "<<Pose[2]<<" \n";
    }
  myfile_oo.close();
}
else cout<<"cannot open the save file for node"<<endl;

 
// Save landmark 
string output_landmark="../VictoriaPark_Data/My_result_VP_lm.txt";
ofstream myfile_2oo (output_landmark);
  
if(myfile_2oo.is_open())  
{
  for (int it=Landmark_offset; it<index;++it)
    {
      const VertexPointXY* NodeLandmark = static_cast<const VertexPointXY*>(GraphOptimizer->vertex(it));
      Vector2d Pose =NodeLandmark->estimate(); 
      myfile_2oo <<Pose[0]<<" "<<Pose[1]<<" \n";
    }
  myfile_2oo.close();
}
else cout<<"cannot open the save file for node"<<endl;

return 0;

}