import matplotlib.pyplot as plt
import numpy as np
import math
from robot import *
import json
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    #Daten vom Koordintensystem:
    angle_Theta=np.pi/2
    twist_alpha=0
    legnth_a=1.2
    offset_d=1.3





    T_id = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]]
    T_id[0][0] = round(math.cos(float(angle_Theta)),5)
    T_id[0][1] = round(-math.sin(float(angle_Theta)) * math.cos(float(twist_alpha)),5)
    T_id[0][2] = round(math.sin(float(angle_Theta)) * math.sin(float(twist_alpha)),5)
    T_id[0][3] = round(float(legnth_a) * math.cos(float(angle_Theta)),5)
    T_id[1][0] = round(math.sin(float(angle_Theta)),5)
    T_id[1][1] = round(math.cos(float(angle_Theta)) * math.cos(float(twist_alpha)),5)
    T_id[1][2] = round(-math.cos(float(angle_Theta)) * math.sin(float(twist_alpha)),5)
    T_id[1][3] = round(float(legnth_a) * math.sin(float(angle_Theta)),5)
    T_id[2][1] = round(math.sin(float(twist_alpha)),5)
    T_id[2][2] = round(math.cos(float(twist_alpha)),5)
    T_id[2][3] = round(float(offset_d),5)

    print(T_id[0][:])
    print(T_id[1][:])
    print(T_id[2][:])
    print("--------")

    g_T_id = np.array([[0],[0],[0],[1]])



    g_T_id = np.dot(b, T_id)

    print(g_T_id)