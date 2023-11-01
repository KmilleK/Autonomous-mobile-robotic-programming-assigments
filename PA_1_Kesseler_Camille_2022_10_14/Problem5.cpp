#include "Complex_graph.h"
#include <vector>
#include <fstream>

using namespace std; 

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
     
    // Creation of the graph 
    PoseGraph graphSlam; 
    
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
                    Matrix3d sigma=MatrixXd::Identity(3,3);
                    //Add a normal odometry edge 
                    graphSlam.AddEdge(index_i,index_j,sigma,stod(data[6].c_str()),stod(data[7].c_str()),stod(data[8].c_str()));

                }
                else if (stoi(data[4].c_str())==1)      //loop closure detection 
                {
                    Matrix3d sigma=MatrixXd::Identity(3,3);
                    //Add a loop closure edge 
                    graphSlam.AddEdge(index_i,index_j,sigma,stod(data[6].c_str()),stod(data[7].c_str()),stod(data[8].c_str()));
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
                    int index_l=stoi(data[3].c_str());

                    Matrix2d sigma=MatrixXd::Identity(2,2);
                    //Add a normal landmark edge 
                    graphSlam.AddEdge(index_i,index_l,sigma,stod(data[6].c_str()),stod(data[7].c_str()));



                }
                else if (stoi(data[2].c_str())==2)   //case of landmark ambiguity 
                {
                    int index_l1=stoi(data[3].c_str());
                    int index_l2=stoi(data[3].c_str());


                    Matrix2d sigma=MatrixXd::Identity(2,2)*2;
                    //Add a less precise landmark edge 
                    graphSlam.AddEdge(index_i,index_l1,sigma,stod(data[6].c_str()),stod(data[7].c_str()));
                
                }
                else
                    cout<<"wrong data in number of landmark"<<endl;
                
            }
            else
                cout<<"wrong data in type"<<endl;
        
    
        }
    }
    else cout<< "cannot open the file: "<<input_data<<endl;
    

    // Initialize the position node 
    ifstream myfile2(input_pos_ini);
    string line2;
    int index2 = 0;
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
        if (index2==0)
            graphSlam.AddNode(index2,x,y,theta,1); 
        else
            graphSlam.AddNode(index2,x,y,theta,0);
        
        index2+=1;

    }
    }
    else cout<< "cannot open the file: "<<input_pos_ini<<endl;
    
    
    // Initialize the landmark 
    ifstream myfile3(input_landmark_ini);
    string line3;
    int index3 = 0;
    if (myfile3.is_open())
    {          
    while(getline(myfile3,line3))
    {   
        //Iterate though each line to fill the pose graph 
        vector<string> data;
        split_string(line3,' ',data);

        double x=stod(data[0].c_str());
        double y=stod(data[1].c_str());
        
        graphSlam.AddNode(index3,x,y,0); 
       
        index3+=1; 

    }
    }
    else cout<< "cannot open the file: "<<input_landmark_ini<<endl;
    
    //plot the initial data 
    
    //optimize 

    double conv_seuil=0.001; 
    int nb_max_iteration=1; 
    graphSlam.optimize(conv_seuil,nb_max_iteration);

    graphSlam.save_landmark();
    graphSlam.save_nodes(); 

    return 0;
}