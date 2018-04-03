import sys
import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 5):
        print "Four arguments required: python kalman2d.py [datafile] [x1] [x2] [lambda]"
        exit()
    
    filename = sys.argv[1]
    x10 = float(sys.argv[2])
    x20 = float(sys.argv[3])
    scaler = float(sys.argv[4])

    # Read data
    lines = [line.rstrip('\n') for line in open(filename)]
    data = []
    for line in range(0, len(lines)):
        data.append(map(float, lines[line].split(' ')))
    # Print out the data
    print "The input data points in the format of 'k [u1, u2, z1, z2]', are:"
    for it in range(0, len(data)):
        print str(it + 1) + " " + str(data[it])

    #initialize data
    x_p = np.matrix([[x10],[x20]])
    I_2 = np.zeros((2,2))
    I_2[0,0] = I_2[1,1] = 1.0
    P_p = scaler * I_2
    Q = np.matrix([[10**(-4),2*(10**(-5))],[2*(10**(-5)),10**(-4)]])
    R = np.matrix([[10**(-2),5*(10**(-3))],[5*(10**(-3)),2*(10**(-2))]])
    Output_x1 = []
    Output_x2 = []
    Output_z1 = []
    Output_z2 = []
    for k in range(0,len(data)):
        line_k = data[k]
        u_p = np.matrix([[line_k[0]],[line_k[1]]])
        z_c = np.matrix([[line_k[2]],[line_k[3]]])
        # Time Update
        x_ps = x_p + u_p
        P_ps = P_p + Q 
        # Measurement update
        K_c  = P_ps * np.linalg.inv(P_ps+R)
        x_c  = x_ps + np.dot(K_c,(z_c - x_ps))
        P_c  = (I_2 - K_c) * P_ps
        Output_x1.append(x_c[0,0])
        Output_x2.append(x_c[1,0])
        Output_z1.append(z_c[0,0])
        Output_z2.append(z_c[1,0])
        P_p = P_c
        x_p = x_c
    plt.figure()
    plt.plot(Output_x1,Output_x2,'r',zorder=1,lw=3)
    plt.scatter(Output_x1,Output_x2, s=40,zorder=2)
    plt.plot(Output_z1,Output_z2,'r',zorder=1,lw=3,color = 'black')
    plt.scatter(Output_z1,Output_z2, s=40,zorder=2)
    plt.title('observation:Black-Orange  Prediction:Red-Blue')
    plt.show()



