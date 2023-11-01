#include <stdio.h>
#include <GL/glut.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

using namespace std;
  
int display_trajectory (string file_name) 
{
    
    ifstream fin(file_name);
    if (!fin) 
    {
        cout << "cannot find trajectory file at " << file_name << endl;  // problem in reading the file
        return 1;
    }
      
    vector<double> x_data;
    vector<double> y_data; 

    while (!fin.eof()) {

      double x,y,theta;
      fin >> x >> y >> theta;
      //cout<<x; 
      x_data.push_back(x);
      y_data.push_back(y);      
    }
  
    
      
    // breadth of picture boundary is 1 pixel
    glPointSize(2.0);    //point size of 1 pixel 

    glClear(GL_COLOR_BUFFER_BIT);
    
    for (int i = 0; i <x_data.size()-1; i ++)
    {
        glBegin(GL_LINES);
        glVertex2i(x_data[i], y_data[i]);
        glVertex2i(x_data[i+1], y_data[i+1]);
        glEnd();
    }
    
    glFlush();

    return 0;
}
  
int display_landmark (string file_name) 
{
    
    ifstream fin(file_name);
    if (!fin) 
    {
        cout << "cannot find trajectory file at " << file_name << endl;  // problem in reading the file
        return 1;
    }
      
    vector<double> x_data;
    vector<double> y_data; 

    while (!fin.eof()) {

      double x,y;
      fin >> x >> y ;
      //cout<<x; 
      x_data.push_back(x);
      y_data.push_back(y);      
    }
  
         
    // breadth of picture boundary is 1 pixel
    glPointSize(3.0);    //point size of 1 pixel 

    //glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_POINTS);
    for (int i = 0; i <x_data.size(); i ++)
    {
        glVertex2i(x_data[i], y_data[i]);
    }
    glEnd();
    glFlush();

    return 0;
}

int main (int argc, char** argv)
{

    string GT_pos="../VictoriaPark_Data/ISAM2_GT_victoriaPark.txt";
    string GT_landmark="../VictoriaPark_Data/ISAM2_GT_victoriaPark_lm.txt";

    string input_pos_ini="../VictoriaPark_Data/My_result_VP.txt";
    string input_landmark_ini="../VictoriaPark_Data/My_result_VP_lm.txt";

    //Basic glut initialisation for window creation 
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
      
    //Initial size and position of the window 
    glutInitWindowSize(1366, 768);
    glutInitWindowPosition(0, 0);
      
    // Window name 
    glutCreateWindow("Ground truth and optimized data");

    // Background color: black (0,0,0) + alpha parameters
    glClearColor(0.0, 0.0, 0.0, 1.0);

    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity();
      
    // setting window dimension in X- and Y- direction
    gluOrtho2D(-100, 220, -80, 280);  // define the viewing region (axis X , axis Y )
    
    // Drawing color (RGB)
    glColor3f(0.0, 1.0, 0.0);
    display_trajectory(GT_pos);

    // Drawing color (RGB)
    glColor3f(1.0,1.0, 1.0);
    display_trajectory(input_pos_ini);

    // Drawing color (RGB)
    glColor3f(1.0,0.0, 0.0);
    display_landmark(GT_landmark);

    // Drawing color (RGB)
    glColor3f(0.0,0.0, 1.0);
    display_landmark(input_landmark_ini);

    glutMainLoop();   // opening the open GL window and keep it open 


}