#include<opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <string>

#include <opencv2/core/eigen.hpp>   // to convert Mat and MatrixXd 
using namespace cv; 
using namespace Eigen; 
using namespace std; 

#define FileName(n)("../dataset/"+std::to_string(n)+".png")


int CleanerMatches(
  //IN
  const Mat& Descriptor,
  const std::vector<DMatch>& Full_matches,
  std::vector<DMatch>& Cleaned_matches
  //OUT 
){
    //compute Max and min distance 

    auto min_max = minmax_element(Full_matches.begin(), Full_matches.end(),[](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    // Remove the bad matching

    for (int i = 0; i < Descriptor.rows; i++) {
    if (Full_matches[i].distance <= max(2*min_dist, 30.0)) {
        Cleaned_matches.push_back(Full_matches[i]);
    }
    }
    
  return 0; 
}


/////////////////////////////////////2.1: Fundamental matrix F , Essential matrix E //////////////////////////////////////
int ComputeFandEMatrix(
    //Input
    const std::vector<Point2d>& ImageLeftPoint_i, 
    const std::vector<Point2d>& ImageRightPoint_i,
    const Matrix3d& K, 
    const Matrix3d& T, 
    //output
    Mat& F,
    Mat& E
){

    // Look at number of Matched point
    int Nb_of_matched;

    if (ImageLeftPoint_i.size()!=ImageRightPoint_i.size() || ImageLeftPoint_i.size()<8 || ImageRightPoint_i.size()<8){
        std::cout<<"Not enought matching point for the Matrix computation"<<std::endl;
        return 1;
    }
    else 
        Nb_of_matched=ImageLeftPoint_i.size();

    //Construct matrix A 
    MatrixXd A = MatrixXd::Zero(Nb_of_matched,9);

    for (int i=0; i<Nb_of_matched;i++)
    {
        //Convert in stable coordinate system
        Vector3d workerLeft_s(ImageLeftPoint_i[i].x,ImageLeftPoint_i[i].y,1);
        Vector3d workerLeft=T*workerLeft_s; 
        Point2d ImageLeftPoint;
        ImageLeftPoint.x=workerLeft[0];
        ImageLeftPoint.y=workerLeft[1];

        Vector3d workerRight_s(ImageRightPoint_i[i].x,ImageRightPoint_i[i].y,1);
        Vector3d workerRight=T*workerRight_s; 
        Point2d ImageRightPoint;
        ImageRightPoint.x=workerRight[0];
        ImageRightPoint.y=workerRight[1];

        A(i,0)=ImageLeftPoint.x*ImageRightPoint.x;
        A(i,1)=ImageLeftPoint.y*ImageRightPoint.x;
        A(i,2)=ImageRightPoint.x;
        A(i,3)=ImageLeftPoint.x*ImageRightPoint.y;
        A(i,4)=ImageLeftPoint.y*ImageRightPoint.y;
        A(i,5)=ImageRightPoint.y;
        A(i,6)=ImageLeftPoint.x;
        A(i,7)=ImageLeftPoint.y;
        A(i,8)=1;
    }

 
    // solve with eigen Af=0
  
    JacobiSVD<MatrixXd> svd;
    svd.compute(A ,ComputeFullV );
       
    VectorXd f;
    f= svd.matrixV().col(8);
    //std::cout<<svd.matrixV()<<std::endl;

    //Reconstruct matrix F from f
    MatrixXd F_eigen = MatrixXd::Zero(3,3);
    F_eigen(0,0)=f[0];
    F_eigen(0,1)=f[1];
    F_eigen(0,2)=f[2];
    F_eigen(1,0)=f[3];
    F_eigen(1,1)=f[4];
    F_eigen(1,2)=f[5];
    F_eigen(2,0)=f[6];
    F_eigen(2,1)=f[7];
    F_eigen(2,2)=f[8];

    // Need to make F rank 2 
    JacobiSVD<MatrixXd> svd_F;
    svd_F.compute(F_eigen,ComputeFullV |ComputeFullU);

    Vector3d SV_F = svd_F.singularValues();
    SV_F(2)=0.0;
    MatrixXd F_Rank2 = MatrixXd::Zero(3,3);
    F_Rank2=svd_F.matrixU()*SV_F.asDiagonal()*svd_F.matrixV().transpose();
    
    // Reconvert on our coordinate system with T 
    MatrixXd F_final = T.transpose()*F_Rank2*T; 
    eigen2cv(F_final,F);

    //Compute matrix E 
   
    MatrixXd E_eigen = MatrixXd::Zero(3,3);
    E_eigen=K.transpose()*F_final*K;

    eigen2cv(E_eigen,E);
    
    return 0;
}

int Estimate_Possible_transformation(
    const Mat& E, 
    std::vector<Matrix3d>& PossibleRotation,
    std::vector<Vector3d>& PossibleTranslation 
)
{
    // first reconvert to eigen matrix for computation . 
MatrixXd E_eigen;
cv2eigen(E,E_eigen);

Matrix3d W;
W << 0,-1,0,1,0,0,0,0,1;

Matrix3d R; 
Vector3d t; 

JacobiSVD<MatrixXd> svd;
svd.compute(E_eigen , ComputeFullV | ComputeFullU );
     
t=svd.matrixU().col(2);
R=svd.matrixU()*W*svd.matrixV().transpose();
PossibleRotation.push_back(R);
PossibleTranslation.push_back(t);

t=-1*svd.matrixU().col(2);
PossibleRotation.push_back(R);
PossibleTranslation.push_back(t);

R=svd.matrixU()*W.transpose()*svd.matrixV().transpose();
PossibleRotation.push_back(R);
PossibleTranslation.push_back(t);

t=svd.matrixU().col(2);
PossibleRotation.push_back(R);
PossibleTranslation.push_back(t);

    return 0; 
}


/////////////////////////////////////2.3: Checking epipolar constraints  //////////////////////////////////////

Point2f pixel2cam(const Point2d &p, const Mat &K) {
  return Point2f
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

int EpipolarConstraints( 
    const vector<Point2d>& pixel1,
    const vector<Point2d>& pixel2, 
    const Mat K,
    const Mat& R, 
    const Mat& t,
    std::vector<double>& Epipolar_constraint)
{
    Mat t_x =
    (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
    t.at<double>(2, 0), 0, -t.at<double>(0, 0),
    -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    for (int i=0;i<pixel1.size();i++) // loop over the match
    { 
        Point2d pt1 = pixel2cam(pixel1[i], K);
        Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
        Point2d pt2 = pixel2cam(pixel2[i], K);
        Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
        Mat d = y2.t()*t_x*R*y1;
        MatrixXd d_eigen; 
        cv2eigen(d,d_eigen);
        Epipolar_constraint.push_back(d_eigen(0,0));
    }        
return 0; 
}
int main()
{
    /////////////////////////////////////2.1: Testing Matrix Computation on image 1/2 //////////////////////////////////////
    
    // From exercise 1: 
    // Read the images with open CV function  stores in class Mat (flag for color image)

    Mat Image0 =imread(FileName(0),IMREAD_COLOR);   
    Mat Image1 =imread(FileName(1),IMREAD_COLOR);


    std::vector<KeyPoint> AllKeypoints0, AllKeypoints1;    // Vector to store the keypoints
    Ptr<FeatureDetector> ORB_detector = ORB::create();     // Open CV detector class to find the features points 
    ORB_detector->detect(Image0,AllKeypoints0);
    ORB_detector->detect(Image1,AllKeypoints1);
    Mat descriptors_0, descriptors_1; 
    Ptr<DescriptorExtractor> ORB_descriptor = ORB::create(); 
    ORB_descriptor->compute(Image0,AllKeypoints0,descriptors_0);
    ORB_descriptor->compute(Image1,AllKeypoints1,descriptors_1);
    std::vector<DMatch> Allmatches;
    Ptr<DescriptorMatcher> ORB_matcher = DescriptorMatcher::create("BruteForce-Hamming");   
    ORB_matcher->match(descriptors_0,descriptors_1,Allmatches);

 
    std::vector<DMatch> good_matches;
    CleanerMatches(descriptors_0,Allmatches,good_matches); 
    
    // Preprocess Data for computing the E / F matrix  

    // image dimension computation 
    int H=Image0.rows;
    int W=Image0.cols; 

    MatrixXd T= MatrixXd::Zero(3,3);
    T<<2.0/W, 0 ,-1.0, 0,  2.0/H ,-1.0, 0, 0,  1;

    // Convert match into vector of point: 
    std::vector<Point2d> pointsLeft;
    std::vector<Point2d> pointsRight; 

    for (int point=0 ; point<(int)good_matches.size();point++){
        pointsLeft.push_back(AllKeypoints0[good_matches[point].queryIdx].pt);
        pointsRight.push_back(AllKeypoints1[good_matches[point].trainIdx].pt);
    }
    //compare result from my function and OpenCV one 
    
    MatrixXd K= MatrixXd::Zero(3,3);
    K(0,0)=520.9;
    K(1,1)=521.0;
    K(2,2)=1;
    K(0,2)=325.1;
    K(1,2)=249.7;

    Mat F, E;

    ComputeFandEMatrix(pointsLeft,pointsRight,K,T,F,E);
    
    std::cout<<"F:"<<std::endl<<F<<std::endl;
    std::cout<<"E:"<<std::endl<<E<<std::endl;
        

    /////////////////////////////////////2.2: Pose of Images //////////////////////////////////////
    std::vector<Matrix4d> AllPose;
    Matrix4d P=MatrixXd::Identity(4,4);

    Ptr<FeatureDetector> ORB_detector_traj = ORB::create();
    Ptr<DescriptorExtractor> ORB_descriptor_traj = ORB::create(); 
    Ptr<DescriptorMatcher> ORB_matcher_traj = DescriptorMatcher::create("BruteForce-Hamming");   

    //Initialisation For Image 0
    Mat Image_i =imread(FileName(0),IMREAD_COLOR);   
   
    std::vector<KeyPoint> AllKeypoints_i, AllKeypoints_i1;    // Vector to store the keypoints
    ORB_detector_traj->detect(Image_i,AllKeypoints_i);
    Mat descriptors_i, descriptors_i1; 
    ORB_descriptor_traj->compute(Image_i,AllKeypoints_i,descriptors_i);
    
    vector<double> Epipo;

    for (int Img_nb=1;Img_nb<132;Img_nb++){
        
        //read new image
        Mat Image_i1 =imread(FileName(Img_nb),IMREAD_COLOR);   
        ORB_detector_traj->detect(Image_i1,AllKeypoints_i1);
        ORB_descriptor_traj->compute(Image_i1,AllKeypoints_i1,descriptors_i1);

        // Match the 2 images
        std::vector<DMatch> Allmatches_traj;
        ORB_matcher_traj->match(descriptors_i,descriptors_i1,Allmatches_traj);

        // remove the bad matching
        std::vector<DMatch> good_matches_traj;
        CleanerMatches(descriptors_i,Allmatches_traj,good_matches_traj);    
    
        // Convert match into vector of point: 
        std::vector<Point2d> pointsLeft_i;
        std::vector<Point2d> pointsRight_i1; 
        for (int point=0 ; point<(int)good_matches_traj.size();point++){
            pointsLeft_i.push_back(AllKeypoints_i[good_matches_traj[point].queryIdx].pt);
            pointsRight_i1.push_back(AllKeypoints_i1[good_matches_traj[point].trainIdx].pt);
        }
        //compare result from my function and OpenCV one 

        // T and K matrix do not change from example above 
        Mat F_i, E_i;

        ComputeFandEMatrix(pointsLeft_i,pointsRight_i1,K,T,F_i,E_i);

        // From Essential matrix to Transformation matrix 

        /*
        //4 possible set of transformation 
        std::vector<Matrix3d> PossibleRotation; 
        std::vector<Vector3d> PossibleTranslation; 
        
        Estimate_Possible_transformation(E_i,PossibleRotation,PossibleTranslation); 

        // Checking which one is the good one with triangulation . 
        std::vector<int> Nb_of_valid_match_vector; 
        for (int possibility=0; possibility<4;possibility++)
        {
            int Nb_of_valid_match=0;

            // triangulate matched point 
            vector<Vector3d> X; 

            //Check their cheirality
            for (int pt=0; pt<X.size();pt++)
            {
                if (PossibleRotation[possibility].row(2)*(X[pt]-PossibleTranslation[possibility])>0)
                {
                    Nb_of_valid_match++; 
                }

            }

            Nb_of_valid_match_vector.push_back(Nb_of_valid_match);
        }
        int index_max=
        */

        Mat R,t,mask,K_oc;
        eigen2cv(K,K_oc); 

       
        recoverPose(E_i,pointsLeft_i,pointsRight_i1,K_oc,R,t,mask);
        
        Matrix3d R_eigen;
        cv2eigen(R,R_eigen);
        P.topLeftCorner(3,3)=R_eigen;
        Vector3d t_eigen; 
        cv2eigen(t,t_eigen);
        P.topRightCorner(3,1)=t_eigen; 
        AllPose.push_back(P);
        
        //Compute epipolar constraints checking 
        std::vector<double> Epipolar_constraint;
        EpipolarConstraints(pointsLeft_i,pointsRight_i1,K_oc,R,t,Epipolar_constraint) ;
        double sum=0;
        for (int k=0;k<Epipolar_constraint.size();k++){
            sum+=Epipolar_constraint[k];
        }
        Epipo.push_back(sum);

        // change memory from i1 to i for next step

        Image_i =Image_i1;   
        AllKeypoints_i= AllKeypoints_i1;    
        descriptors_i= descriptors_i1; 
       
        
    }

    for (int i=0;i<Epipo.size();i++){
    cout<<"Pose compute: "<<endl<<AllPose[i]<<endl;
    cout<<"Epipolar Constraint sum image"<<i << "="<<Epipo[i]<<endl;
    }


    return 0; 
}