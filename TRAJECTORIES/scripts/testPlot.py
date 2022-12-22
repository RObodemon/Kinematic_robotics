import numpy as np
import matplotlib.pyplot as plt

def main():

    file_data = np.loadtxt('C:/Users/Henyue Guan/Desktop/DSC/Robot/Robotics Arm/Project/Trajectories/data_plot/linear_trj.txt')
    t = file_data[:,0]
    p = file_data[:,1]

    plt.plot(t,p)
    plt.show()


if __name__ == "__main__":
    main()
