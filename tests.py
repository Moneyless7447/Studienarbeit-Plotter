import matplotlib.pyplot as plt
import numpy as np
import math
from robot import *
import json
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    #Daten vom Koordintensystem:
    # 1
    # angle_Theta=np.pi/2
    # twist_alpha=0
    # length_a=1.2
    # offset_d=1.3

    # 2
    # angle_Theta = np.pi
    # twist_alpha = 0
    # length_a = 2.4
    # offset_d = 3

    # 3
    # angle_Theta = 0
    # twist_alpha = np.pi
    # length_a = 2.3
    # offset_d = 1

    # 4
    # angle_Theta = np.pi/4
    # twist_alpha = -np.pi/4
    # length_a = 1.5
    # offset_d = 0.2

    # 5
    # angle_Theta = 0
    # twist_alpha = np.pi/4
    # length_a = 1
    # offset_d = 3

    # 6
    angle_Theta = -np.pi/2
    twist_alpha = 0
    length_a = 0.5
    offset_d = 2


    T_id = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]]
    T_id[0][0] = round(math.cos(float(angle_Theta)),5)
    T_id[0][1] = round(-math.sin(float(angle_Theta)) * math.cos(float(twist_alpha)),5)
    T_id[0][2] = round(math.sin(float(angle_Theta)) * math.sin(float(twist_alpha)),5)
    T_id[0][3] = round(float(length_a) * math.cos(float(angle_Theta)),5)
    T_id[1][0] = round(math.sin(float(angle_Theta)),5)
    T_id[1][1] = round(math.cos(float(angle_Theta)) * math.cos(float(twist_alpha)),5)
    T_id[1][2] = round(-math.cos(float(angle_Theta)) * math.sin(float(twist_alpha)),5)
    T_id[1][3] = round(float(length_a) * math.sin(float(angle_Theta)),5)
    T_id[2][1] = round(math.sin(float(twist_alpha)),5)
    T_id[2][2] = round(math.cos(float(twist_alpha)),5)
    T_id[2][3] = round(float(offset_d),5)

    # T_id2 = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]]
    # T_id2[0][0] = round(math.cos(float(angle_Theta2)), 5)
    # T_id2[0][1] = round(-math.sin(float(angle_Theta2)) * math.cos(float(twist_alpha2)), 5)
    # T_id2[0][2] = round(math.sin(float(angle_Theta2)) * math.sin(float(twist_alpha2)), 5)
    # T_id2[0][3] = round(float(length_a2) * math.cos(float(angle_Theta2)), 5)
    # T_id2[1][0] = round(math.sin(float(angle_Theta2)), 5)
    # T_id2[1][1] = round(math.cos(float(angle_Theta2)) * math.cos(float(twist_alpha2)), 5)
    # T_id2[1][2] = round(-math.cos(float(angle_Theta2)) * math.sin(float(twist_alpha2)), 5)
    # T_id2[1][3] = round(float(length_a2) * math.sin(float(angle_Theta2)), 5)
    # T_id2[2][1] = round(math.sin(float(twist_alpha2)), 5)
    # T_id2[2][2] = round(math.cos(float(twist_alpha2)), 5)
    # T_id2[2][3] = round(float(offset_d2), 5)


    print(T_id[0][:])
    print(T_id[1][:])
    print(T_id[2][:])
    print("--------")

    a = [[0], [0], [0], [1]]
    x = [[1], [0], [0], [1]]
    y = [[0], [1], [0], [1]]
    z = [[0], [0], [1], [1]]

    #T_id = np.dot(T_id,T_id2)
    abc = np.dot(T_id,a)
    abcx = np.dot(T_id,x)
    abcy = np.dot(T_id, y)
    abcz = np.dot(T_id, z)

    print(f"abc: {abc}")
    print(f"x: {abcx}")
    print(f"y: {abcy}")
    print(f"z: {abcz}")