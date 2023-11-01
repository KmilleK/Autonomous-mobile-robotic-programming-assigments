import matplotlib.pyplot as plt
import numpy as np
import os



def final_plot(GT,map,traj):

    fig, ax = plt.subplots(figsize=(5, 5))
    plt.xlim([-1, 12])
    plt.ylim([-1,12])

    #Read the file ground truth 
    GT =np.loadtxt(GT)
    for l in GT :
        #print(l[2])
        x_l=l[1]
        y_l=l[2]
        circle=plt.Circle((x_l,y_l),radius=0.1,color='r')
        ax.add_patch(circle)

    #Read my map file 

    My_map =np.loadtxt(map)
    for l in My_map :
        x_l=l[1]
        y_l=l[2]
        circle=plt.Circle((x_l,y_l),radius=0.1,color='b')
        ax.add_patch(circle)


    #Read the trajectory file 
    my_traj=np.loadtxt(traj)
    plt.plot(my_traj[:,0],my_traj[:,1])


    ax.set(xlabel='x', ylabel='y',
           title="Final map and trajectory")
    ax.grid()
    
    fig.savefig("plot/Final.png")
    plt.show()

    
def RMSE_plot(data):
    rmse=np.loadtxt(data)
    fig, ax = plt.subplots(figsize=(5, 5))

    plt.plot(rmse[:,0],rmse[:,1])
    ax.set(xlabel='step', ylabel='Rmse',
           title="RMSE evolution during EKF")
    fig.savefig("plot/rmse")
    plt.show()  


def instant_plot(data):
    initial_count =0
    for files in os.listdir("./plot/Instant_dat"):
        initial_count+=1
    print(initial_count)
    
    for  i in range(0,initial_count-1):
        name=data
        name+="{}.dat".format(i)
        All_data=np.loadtxt(name)
        fig, ax = plt.subplots(figsize=(5, 5))
        plt.xlim([-1, 12])
        plt.ylim([-1,12])       
        t1 = plt.arrow(All_data[0,0], All_data[0,1], np.cos(All_data[0,2]), np.sin(All_data[0,2]),width=0.1)
        plt.gca().add_patch(t1)
        #landmark plot 
        nb_landmark=All_data[:,0].size-2
        
        for k in range(1,nb_landmark+1):
                x_l=All_data[k,1]
                y_l=All_data[k,2]
                circle=plt.Circle((x_l,y_l),radius=0.1,color='b')
                ax.add_patch(circle)

        ax.set(xlabel='x', ylabel='y',title="step{}".format(i))

        ax.grid()
        fig.savefig("plot/Instant_image/step{}.png".format(i))
        plt.close(fig)

    




        

# Data Reading for plot 


GT_map='data/world.dat'
my_map="bin/MyMap.dat"
my_traj="bin/MyTraj.dat"
my_rmse="bin/MyRMSE.dat"
All_steps="plot/Instant_dat/step"



final_plot(GT_map,my_map,my_traj)

RMSE_plot(my_rmse)

instant_plot(All_steps)