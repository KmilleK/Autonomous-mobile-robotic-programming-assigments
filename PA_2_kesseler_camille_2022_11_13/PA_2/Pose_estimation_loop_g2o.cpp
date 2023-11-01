/*
reference for 
http://wangxinliu.com/slam/optimization/research&study/g2o-4/

https://fzheng.me/2016/03/15/g2o-demo/

https://vincentqin.gitee.io/blogresource-4/slam/g2o-details.pdf
*/
// need include

#include <iostream>
#include <cmath>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include "se2.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>

#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include "g2o/core/optimization_algorithm_levenberg.h"
#include <Eigen/Core>

#include <g2o/core/g2o_core_api.h>

#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"

using namespace std; 
using namespace g2o; 
using namespace Eigen;

//Function to convert angle from degre to rad and inversely
double DegToRad(double angle)
{
  return angle*(M_PI/180);
}
double RadToDeg(double angle)
{
  return angle*(180/M_PI);
}


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


class EdgeSE2PointXY_angle : public BaseBinaryEdge<2,Eigen::Vector2d, VertexSE2, VertexPointXY>
{
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    void computeError()
    {
        const VertexSE2* Pose = static_cast<const VertexSE2*>(_vertices[0]); 
        const VertexPointXY* landmark = static_cast<const VertexPointXY*>(_vertices[1]);
        
        //compute estimate : convert from xy to angle/distance         
        
        Eigen::Vector2d estimate;
        
        Vector3d x=Pose->estimate().toVector(); 
        Vector2d l=landmark->estimate(); 
        
        estimate[0]=sqrt((l[0]-x[0])*(l[0]-x[0])+(l[1]-x[1])*(l[1]-x[1]));
        estimate[1]=atan2(l[1]-x[1],l[0]-x[0])-x[2];
        
         
        //to do 
        _error =  estimate -_measurement; 
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


int main()
{

// Optimizer definition  
cout<< "start optimization"<<endl;
g2o::SparseOptimizer* GraphOptimizer=  new g2o::SparseOptimizer; 

// solver definition 
typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;

auto solver = g2o::make_unique<LinearSolverEigen<SlamBlockSolver::PoseMatrixType>>(); 

// optimization algorithm: 

g2o::OptimizationAlgorithmLevenberg* optiAlgo = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<SlamBlockSolver>(std::move(solver))); 

// setup the optimizer 

GraphOptimizer->setAlgorithm(optiAlgo); 

//Fill the graph model  with nodes and edges 

// Poses vertices addition
    
//Pose 0 SE2
const SE2& pose= SE2(0,0,DegToRad(0)); 
VertexSE2* Nodepose = new VertexSE2; 
Nodepose->setId(0);
Nodepose->setEstimate(pose); 
GraphOptimizer->addVertex(Nodepose); 

//Pose 1
const SE2& pose1= SE2(19,9.5,DegToRad(88)); 
VertexSE2* Nodepose1 = new VertexSE2; 
Nodepose1->setId(1);
Nodepose1->setEstimate(pose1); 
GraphOptimizer->addVertex(Nodepose1); 

//Pose 2 
const SE2& pose2= SE2(19,18,DegToRad(92)); 
VertexSE2* Nodepose2 = new VertexSE2; 
Nodepose2->setId(2);
Nodepose2->setEstimate(pose2); 
GraphOptimizer->addVertex(Nodepose2); 

//Pose 3 
const SE2& pose3= SE2(0,21,DegToRad(170)); // how to define SE2
VertexSE2* Nodepose3 = new VertexSE2; 
Nodepose3->setId(3);
Nodepose3->setEstimate(pose3); 
GraphOptimizer->addVertex(Nodepose3); 

//Pose 4 
const SE2& pose4= SE2(5,5,DegToRad(0)); // how to define SE2
VertexSE2* Nodepose4 = new VertexSE2; 
Nodepose4->setId(4);
Nodepose4->setEstimate(pose4); 
GraphOptimizer->addVertex(Nodepose4); 

// Edges btw poses 
//edge 01
EdgeSE2 *Measurement01 = new EdgeSE2;
Measurement01->vertices()[0]=GraphOptimizer->vertex(0);
Measurement01->vertices()[1]=GraphOptimizer->vertex(1);
const SE2& measure01= SE2(20,10,DegToRad(90)); // how to define SE2
Measurement01->setMeasurement(measure01);    // how to define 
Measurement01->setInformation(Eigen::Matrix3d::Identity());
GraphOptimizer->addEdge(Measurement01); 
 
 
//edge 12
 EdgeSE2* Measurement12 = new EdgeSE2;
 Measurement12->vertices()[0]=GraphOptimizer->vertex(1);
 Measurement12->vertices()[1]=GraphOptimizer->vertex(2);
 const SE2& measure12= SE2(10,0,DegToRad(0)); // how to define SE2
 Measurement12->setMeasurement(measure12);    // how to define 
 Measurement12->setInformation(Eigen::Matrix3d::Identity());
 GraphOptimizer->addEdge(Measurement12);
 
 //edge 23
  EdgeSE2* Measurement23 = new EdgeSE2;
  Measurement23->vertices()[0]=GraphOptimizer->vertex(2);
  Measurement23->vertices()[1]=GraphOptimizer->vertex(3);
  const SE2& measure23= SE2(0,20,DegToRad(90)); // how to define SE2
  Measurement23->setMeasurement(measure23);    // how to define 
  Measurement23->setInformation(Eigen::Matrix3d::Identity());
  GraphOptimizer->addEdge(Measurement23);
   

   //edge 34
  EdgeSE2* Measurement34 = new EdgeSE2;
  Measurement34->vertices()[0]=GraphOptimizer->vertex(3);
  Measurement34->vertices()[1]=GraphOptimizer->vertex(4);
  const SE2& measure34= SE2(0,19,DegToRad(175)); // how to define SE2
  Measurement34->setMeasurement(measure34);    // how to define 
  Measurement34->setInformation(Eigen::Matrix3d::Identity());
  GraphOptimizer->addEdge(Measurement34);

   //edge 40
  EdgeSE2* Measurement40 = new EdgeSE2;
  Measurement40->vertices()[0]=GraphOptimizer->vertex(4);
  Measurement40->vertices()[1]=GraphOptimizer->vertex(0);
  const SE2& measure40= SE2(0,0,DegToRad(0)); // how to define SE2
  Measurement40->setMeasurement(measure40);    // how to define 
  Measurement40->setInformation(Eigen::Matrix3d::Identity());
  GraphOptimizer->addEdge(Measurement40); 
// Landmark vertices addition 

//landmark 0
VertexPointXY* landmark0 = new VertexPointXY;
landmark0->setId(10);
landmark0->setEstimate(Eigen::Vector2d(7,24));      // check if right 
GraphOptimizer->addVertex(landmark0); 

//landmark 1
VertexPointXY* landmark1 = new VertexPointXY;
landmark1->setId(11);
landmark1->setEstimate(Eigen::Vector2d(15,29));      // check if right 
GraphOptimizer->addVertex(landmark1);

// Edges btw pose & landmark 

//observation 0 to 0 
EdgeSE2PointXY_angle* landmarkObservation00 = new EdgeSE2PointXY_angle;  // to change as the measurement model is different 
landmarkObservation00->vertices()[0]=GraphOptimizer->vertex(0);
landmarkObservation00->vertices()[1]=GraphOptimizer->vertex(10);
landmarkObservation00->setMeasurement(Eigen::Vector2d(26,DegToRad(70)));
landmarkObservation00->setInformation(Eigen::Matrix2d::Identity());
GraphOptimizer->addEdge(landmarkObservation00);

//observation 0 to 1 
EdgeSE2PointXY_angle* landmarkObservation01 = new EdgeSE2PointXY_angle;  // to change as the measurement model is different 
landmarkObservation01->vertices()[0]=GraphOptimizer->vertex(0);
landmarkObservation01->vertices()[1]=GraphOptimizer->vertex(11);
landmarkObservation01->setMeasurement(Eigen::Vector2d(31,DegToRad(60)));
landmarkObservation01->setInformation(Eigen::Matrix2d::Identity());
GraphOptimizer->addEdge(landmarkObservation01);

//observation 1 to 0 
EdgeSE2PointXY_angle* landmarkObservation10 = new EdgeSE2PointXY_angle;  // to change as the measurement model is different 
landmarkObservation10->vertices()[0]=GraphOptimizer->vertex(1);
landmarkObservation10->vertices()[1]=GraphOptimizer->vertex(10);
landmarkObservation10->setMeasurement(Eigen::Vector2d(19,DegToRad(35)));
landmarkObservation10->setInformation(Eigen::Matrix2d::Identity());
GraphOptimizer->addEdge(landmarkObservation10);

//observation 1 to 1 
EdgeSE2PointXY_angle* landmarkObservation11 = new EdgeSE2PointXY_angle;  // to change as the measurement model is different 
landmarkObservation11->vertices()[0]=GraphOptimizer->vertex(1);
landmarkObservation11->vertices()[1]=GraphOptimizer->vertex(11);
landmarkObservation11->setMeasurement(Eigen::Vector2d(19,DegToRad(15)));
landmarkObservation11->setInformation(Eigen::Matrix2d::Identity());
GraphOptimizer->addEdge(landmarkObservation11);


//observation 2 to 0 
EdgeSE2PointXY_angle* landmarkObservation20 = new EdgeSE2PointXY_angle;  // to change as the measurement model is different 
landmarkObservation20->vertices()[0]=GraphOptimizer->vertex(2);
landmarkObservation20->vertices()[1]=GraphOptimizer->vertex(10);
landmarkObservation20->setMeasurement(Eigen::Vector2d(13,DegToRad(70)));
landmarkObservation20->setInformation(Eigen::Matrix2d::Identity());
GraphOptimizer->addEdge(landmarkObservation20);

//observation 2 to 1 
EdgeSE2PointXY_angle* landmarkObservation21 = new EdgeSE2PointXY_angle;  // to change as the measurement model is different 
landmarkObservation21->vertices()[0]=GraphOptimizer->vertex(2);
landmarkObservation21->vertices()[1]=GraphOptimizer->vertex(11);
landmarkObservation21->setMeasurement(Eigen::Vector2d(10,DegToRad(40)));
landmarkObservation21->setInformation(Eigen::Matrix2d::Identity());
GraphOptimizer->addEdge(landmarkObservation21);

// fixed the first point for r

VertexSE2* InitialPoseRobot = dynamic_cast<VertexSE2*>(GraphOptimizer->vertex(0));
InitialPoseRobot->setFixed(true);

// optimization results: 

GraphOptimizer->setVerbose(true);  // print intermediate information

// initialize and run the optimizer
int nbIterations= 10; 

GraphOptimizer->initializeOptimization(); 
GraphOptimizer->optimize(nbIterations);

cout<<"optimization done"<<endl;
//print the result of the optimization

    //SE2 estimateNode = Nodepose->estimate();
    Vector3d Pose_0 =Nodepose->estimate().toVector();
    Vector3d Pose_1 =Nodepose1->estimate().toVector();
    Vector3d Pose_2 =Nodepose2->estimate().toVector();
    Vector3d Pose_3 =Nodepose3->estimate().toVector();
    Vector3d Pose_4 =Nodepose4->estimate().toVector();
    std::cout <<"Pose 0 x: "<< Pose_0[0]<<" y: "<< Pose_0[1]<< " theta: "<<RadToDeg(Pose_0[2])<<std::endl;
    std::cout <<"Pose 1 x: "<< Pose_1[0]<<" y: "<< Pose_1[1]<< " theta: "<<RadToDeg(Pose_1[2])<<std::endl;
    std::cout <<"Pose 2 x: "<< Pose_2[0]<<" y: "<< Pose_2[1]<< " theta: "<<RadToDeg(Pose_2[2])<<std::endl;
    std::cout <<"Pose 3 x: "<< Pose_3[0]<<" y: "<< Pose_3[1]<< " theta: "<<RadToDeg(Pose_3[2])<<std::endl;
    std::cout <<"Pose 4 x: "<< Pose_4[0]<<" y: "<< Pose_4[1]<< " theta: "<<RadToDeg(Pose_4[2])<<std::endl;

    Vector2d Landmark_0=landmark0->estimate();
    Vector2d Landmark_1=landmark1->estimate();
    std::cout <<"Landmark 1 x: "<< Landmark_0[0]<<" y: "<< Landmark_0[1]<< std::endl;
    std::cout <<"Landmark 2 x: "<< Landmark_1[0]<<" y: "<< Landmark_1[1]<< std::endl;

return 0; 
}
