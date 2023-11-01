
#include<opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <vector>
/*
  https://docs.opencv.org/3.4/d1/d89/tutorial_py_orb.html
 */

using namespace cv; 

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

int main(){
    
    // Read the images with open CV function  stores in class Mat (flag for color image)

    Mat Image0 =imread("../dataset/2.png",IMREAD_COLOR);   
    Mat Image1 =imread("../dataset/1.png",IMREAD_COLOR);

    /////////////////////////////////////1.1: Visualize it //////////////////////////////////////

    // KeyPoint Initialisation

    std::vector<KeyPoint> AllKeypoints0, AllKeypoints1;    // Vector to store the keypoints
    Ptr<FeatureDetector> ORB_detector = ORB::create();     // Open CV detector class to find the features points 

    // Find the featured point 

    ORB_detector->detect(Image0,AllKeypoints0);
    ORB_detector->detect(Image1,AllKeypoints1);

    // Initialize the descriptor

    Mat descriptors_0, descriptors_1; 
    Ptr<DescriptorExtractor> ORB_descriptor = ORB::create(); 
    
    // Compute the descriptor 

    ORB_descriptor->compute(Image0,AllKeypoints0,descriptors_0);
    ORB_descriptor->compute(Image1,AllKeypoints1,descriptors_1);


    // show the results images

    Mat ImageShowKeypoint0, ImageShowKeypoint1;

    drawKeypoints(Image0,AllKeypoints0,ImageShowKeypoint0,Scalar::all(-1),DrawMatchesFlags::DEFAULT);  // Image, keypoints, outImage, color, flags
    imshow("Image 0 Keypoints",ImageShowKeypoint0);
    drawKeypoints(Image1,AllKeypoints1,ImageShowKeypoint1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);  // Image, keypoints, outImage, color, flags
    imshow("Image 1 Keypoints",ImageShowKeypoint1);
    waitKey(0);


    /////////////////////////////////////1.2: Matching it  //////////////////////////////////////

    
    // Initialize matcher for features

    std::vector<DMatch> Allmatches;
    Ptr<DescriptorMatcher> ORB_matcher = DescriptorMatcher::create("BruteForce-Hamming");    //https://docs.opencv.org/3.4/db/d39/classcv_1_1DescriptorMatcher.html
    
    // Matching the features 

    ORB_matcher->match(descriptors_0,descriptors_1,Allmatches);

    // Show the resulting matches 
    Mat Image_Ouput_matches;
    drawMatches(Image0,AllKeypoints0,Image1,AllKeypoints1,Allmatches,Image_Ouput_matches);
    imshow("All matched keypoints between image 0 & image 1",Image_Ouput_matches);
    waitKey(0);

    // Outliers Management as too many spurious matches 

    std::vector<DMatch> Final_matches;
    CleanerMatches(descriptors_0,Allmatches,Final_matches);

    Mat Image_Final_matches;
    drawMatches(Image0,AllKeypoints0,Image1,AllKeypoints1,Final_matches,Image_Final_matches);
    imshow("All cleaned matched keypoints between image 0 & image 1",Image_Final_matches);
    waitKey(0);

    

    
    
    return 0;

}