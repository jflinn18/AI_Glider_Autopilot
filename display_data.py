import serial, copy, math, pdb

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

ser = serial.Serial('/dev/ttyUSB0', 115200)

#indecies = {0:[1,3,4],1:[0,2,5],2:[1,3,6],3:[0,2,7],4:[0,5,7],5:[4,6,1],6:[5,7,2],7:[4,6,3]}

indecies = {0:[1,3],1:[0,2],2:[1,3],3:[0,2]}

#points = np.array([[1,-0.5,0],[1,0.5,0],[-1,0.5,0],[-1,-0.5,0],[1,-0.5,0.5],[1,0.5,0.5],[-1,0.5,0.5],[-1,-0.5,0.5]])

points = np.array([[1,-0.5,0],[1,0.5,0],[-1,0.5,0],[-1,-0.5,0]])

#points = points.transpose()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


while True:
    ser_data = ser.readline().decode("utf-8").split(" ")
    heading, pitch, roll = float(ser_data[0]), float(ser_data[1]), float(ser_data[2][:-2])

    temp_pts = copy.deepcopy(points)
    sin_roll = math.sin(roll)
    cos_roll = math.cos(roll)
    m_roll = np.array([[1,0,0],[0, cos_roll, -sin_roll],[0, sin_roll, cos_roll]])

    #points = copy.deepcopy(np.matmul(temp_pts, m_roll))
    temp_pts = np.matmul(temp_pts, m_roll)

    #temp_pts = copy.deepcopy(points)
    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)
    m_pitch = np.array([[cos_pitch, 0, sin_pitch],[0,1,0],[-sin_pitch, 0, cos_pitch]])

    #points = copy.deepcopy(np.matmul(temp_pts, m_pitch))
    temp_pts = np.matmul(temp_pts, m_pitch)
    
    ax.clear()
    for i in range(len(points)):
        temp = []
        for j in indecies[i]:
            #temp.append(points[j])
            temp.append(temp_pts[j])
        adj_pts = np.array(temp).transpose()
        #pdb.set_trace()
        
        for j in range(len(temp)):
            #ax.plot([points[i][0], adj_pts[0][j]], [points[i][1], adj_pts[1][j]], [points[i][2], adj_pts[2][j]])
            ax.plot([temp_pts[i][0], adj_pts[0][j]], [temp_pts[i][1], adj_pts[1][j]], [temp_pts[i][2], adj_pts[2][j]])
    plt.draw()
    plt.pause(0.001)

