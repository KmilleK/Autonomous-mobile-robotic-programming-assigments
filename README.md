<h1 align="center">Autonomous-mobile-robotic-programming-assigments</h1>

The different programming assignments realized in C++ during the autonomous mobile robotic course at SNU

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">TO DO</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
---
## About the project 

This project is 5 different programming assignments on the autonomous mobile robotic topic. Each of them can be seen separately and precise descriptions of the code are given in the pdf in the associate folder.   

---
## PA1:  Pose graph optimization basic concept 

### Built with

* Basic CPP code

### Concept, solution, and dataset for test

Recoding a pose graph optimization process from scratch with: 
	-Code structure with position node and measurement edge 
	-Levenberg-Marquard algorithm for resolution

 We apply our solution to the following problem: 
   - 4 robot positions and 3 measurements
   - 4 robot positions, 3 measurements, and 1 loop closure measurement
   - 4 robot positions, 3 measurements, and 2 landmarks with their measurements from the robot
   - 4 robot positions, 3 measurements, 1 loop closure, and 2 landmarks with their measurements from the robot
   - Victoria Park dataset

---
## PA2: Pose graph optimization with basic open-source solvers

### Built with

* Main code in cpp
* Ceres solver library
* g2o solver library

### Concept, solution, and dataset for test

Use the open-source pose graph solver Ceres and g2o to resolve our problem with:
  - redefine the measurement error function

We apply our solution to the following problem: 
   - 4 robot positions, 3 measurements, 1 loop closure, and 2 landmarks with their measurements from the robot
   - Victoria Park dataset

---
## PA3:  Feature extraction and epipolar geometry

### Built with

* Main code in cpp
* OpenCV library for image and descriptor
* Eigen3 library for matrix and vector mathematics

### Concept, solution, and dataset for test

Calculate with the OpenCV library the ORB feature in different images and then match them.
From these matched points we can use epipolar geometry to estimate the relative pose of each image and so the trajectory

---
## PA4: 2D Extended Kalman Filter

### Built with

* Main code in cpp
* Eigen3 library for matrix and vector mathematics

### Concept, solution, and dataset for test

Use the data and template code given by the TA to test a 2D Kalman filter. The sensor data are given in terms of odometry (robot movement) and sensor (landmark detection). 
The extended Kalman filter can be decomposed in two steps: prediction and correction.

---
## PA5: Projection factor implementation for Bundle adjustment

### Built with

* Main code in cpp
* Ceres solver for the resolution
* Eigen3 library for matrix and vector mathematics
* Python for visualization
* 
### Concept, solution, and dataset for test

Given a prefilled bundle adjustment project, we define the projection factor to solve it and compute the RMSE for the test on a dataset 




