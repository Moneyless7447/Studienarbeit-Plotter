import matplotlib.pyplot as plt
import numpy as np
import math
from robot import *
import json
from mpl_toolkits import mplot3d


#Datei auslesen und als Dictionary abspeichern
def jsontoDic(file_name):
    #Datei oeffnen
    robot_json = open(file_name)

    # JSON Object als Dictionary
    robot_data = json.load(robot_json)

    #print(robot_data)

    #for i in data['robot']:
     #   print(i)

    robot_json.close()
    return robot_data

#Sucht nach "key" in gegebener Liste, return: Anzahl der gefundenen Keys
def searchForKeyinDic(dic, key_search):
    #TODO: Code
    anz_key = 0

    return anz_key

# Umstrukturierung des Dictionaries für Weiterverwendung (nicht fertig!!!)
def restructureDic(dic):
    # TODO: Code
    rob_dic = {}
    anz_joints = 0
    #for 'robot' in dic:

        #anz_joints += 1

    #print(f"anz_joints: {anz_joints}")
"""
Geg.: {'robot': [{'angle': 'Winkel', 'length': '1', 'offset': '2', 'twist': '3', 'children': [{'angle': 'a', 'length': 'b', 'offset': 'd', 'twist': 'd'}]}]}
Ziel: joints = {1: {'name': '1', 'parent': '0', 'angle': 'Winkel', 'length': '1', 'offset': '2', 'twist': '3'},
                2: {'name': '2', 'parent': '1', 'angle': 'a', 'length': 'b', 'offset': 'd', 'twist': 'd'}}
'parent': 'ID in joints, where 0 equals given base'

-----------------------------------------------------------------------------------------------------------------------

Geg.: 
{'robot': [{'angle': '11', 'length': '12', 'offset': '13', 'twist': '14', 'children':          (1)
[{'angle': '111', 'length': '112', 'offset': '113', 'twist': '114', 'children':                (3)
[{'angle': '1111', 'length': '1112', 'offset': '1113', 'twist': '1114'},                       (5)
{'angle': '1121', 'length': '1122', 'offset': '1123', 'twist': '1124'}]},                      (6)
{'angle': '121', 'length': '122', 'offset': '123', 'twist': '124'}]},                          (4)
{'angle': '21', 'length': '22', 'offset': '23', 'twist': '24'}]}                               (2)

Ziel: joints = {1: {'name': '1', 'parent': '0', 'angle': '11', 'length': '12', 'offset': '13', 'twist': '14'},
                2: {'name': '2', 'parent': '0', 'angle': '21', 'length': '22', 'offset': '23', 'twist': '24'},
                3: {'name': '1.1', 'parent': '1', 'angle': '111', 'length': '112', 'offset': '113', 'twist': '114'},
                4: {'name': '1.2', 'parent': '1', 'angle': '121', 'length': '122', 'offset': '123', 'twist': '124'},
                5: {'name': '1.1.1', 'parent': '3', 'angle': '1111', 'length': '1112', 'offset': '1113', 'twist': '1114'},
                6: {'name': '1.1.2', 'parent': '3', 'angle': '1121', 'length': '1122', 'offset': '1123', 'twist': '1124'}}

-----------------------------------------------------------------------------------------------------------------------
a = joints[1].get('twist').get('naaame')


"""


# Beispiel B Dictionary für einen Roboter
joints = {1: {'name': '1', 'parent': '0', 'angle': '11', 'length': '12', 'offset': '13', 'twist': '14'},
          2: {'name': '2', 'parent': '0', 'angle': '21', 'length': '22', 'offset': '23', 'twist': '24'},
          3: {'name': '1.1', 'parent': '1', 'angle': '111', 'length': '112', 'offset': '113', 'twist': '114'},
          4: {'name': '1.2', 'parent': '1', 'angle': '121', 'length': '122', 'offset': '123', 'twist': '124'},
          5: {'name': '1.1.1', 'parent': '3', 'angle': '1111', 'length': '1112', 'offset': '1113', 'twist': '1114'},
          6: {'name': '1.1.2', 'parent': '3', 'angle': '1121', 'length': '1122', 'offset': '1123', 'twist': '1124'}}


# Liste der nötigen Transformationsmatrizen, um vom Ursprung zu jedem Gelenk transformieren zu können
def init_kin_chains():
    """
    Eine Liste aus Listen für Gelenk 1 bis n mit der kinematischen Kette für das jeweilige Gelenk
    bis zum Basiskoordinatensystem.
    Ziel, zu Beispiel B, Index in aeusserer Liste entspricht ID in Dictionary (1-6):
    kin_chain_list = [[0], [0,1], [0,2], [0,1,3], [0,1,4], [0,1,3,5], [0,1,3,6]]

    """
    kin_chain_list = [[i] for i in range(len(joints)+1)]
    for n in range(len(kin_chain_list)):
        tmp_parent = n

        while int(kin_chain_list[n][0]) != 0:
            kin_chain_list[n] = [int((joints[tmp_parent].get('parent'))), *kin_chain_list[n]]
            tmp_parent = int(joints[tmp_parent].get('parent'))

    print("kin_chain_list: ", kin_chain_list)

    return kin_chain_list

# Gibt eine Transformationsmatrix nach DH Konvention für T_id-1_id aus Daten von Dictionary joints zurueck
def init_transformationmatrix_dh(id):
    #T_id = DH Transformationsmatrix T_id-1_id
    T_id = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]]
    T_id[0][0] = math.cos(int(joints[id].get('angle')))
    T_id[0][1] = -math.sin(int(joints[id].get('angle')))*math.cos(int(joints[id].get('twist')))
    T_id[0][2] = math.sin(int(joints[id].get('angle')))*math.sin(int(joints[id].get('twist')))
    T_id[0][3] = int(joints[id].get('length'))*math.cos(int(joints[id].get('angle')))
    T_id[1][0] = math.sin(int(joints[id].get('angle')))
    T_id[1][1] = math.cos(int(joints[id].get('angle'))) * math.cos(int(joints[id].get('twist')))
    T_id[1][2] = -math.cos(int(joints[id].get('angle'))) * math.sin(int(joints[id].get('twist')))
    T_id[1][3] = int(joints[id].get('length')) * math.sin(int(joints[id].get('angle')))
    T_id[2][1] = math.sin(int(joints[id].get('twist')))
    T_id[2][2] = math.cos(int(joints[id].get('twist')))
    T_id[2][3] = int(joints[id].get('offset'))

    return T_id

# Gibt eine Gesamtransformationsmatrix_0_Gelenk für EIN Gelenk aus der kin_chain_list zurueck
def init_chain_transformationmatrix(id):
    #Segment aus kin_chain_list abhaengig von ID
    piece_kin_chain_list=kin_chain_list[id]
    g_T_id = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    for i in range(1, len(piece_kin_chain_list)):
            g_T_id = np.dot(g_T_id, init_transformationmatrix_dh(piece_kin_chain_list[i]))

    return g_T_id

# Gibt eine Liste aus Gesamtransformationsmatrizen für alle Gelenke zurueck
def init_entire_transformationmatrices():
    #Ziel: [[MatrixG0], [MatrixG1], [MatrixG2], ...]
    entire_T_list = [init_chain_transformationmatrix(i) for i in range(len(kin_chain_list))]

    return entire_T_list


def points_coord_system(id):
    ursprung_punkt_show = np.array([[0], [0], [0], [1]])
    


    return ursprung_punkt_show




if __name__ == '__main__':


    robot_data_dic = jsontoDic('example_robot_dh.json')
    #print(robot_data_dic)
    robot_data_dic_backup = robot_data_dic

    #print(robot_data_dic.get('angle'))
    #print(robot_data_dic.values())
    ##print(robot_data_dic[0]['angle'])
    #robot_data_dic[0] = {}

    restructureDic(robot_data_dic)
    kin_chain_list = init_kin_chains()

    #print(f'T_id: \n {init_transformationmatrix_dh(2)[0]} \n {init_transformationmatrix_dh(2)[1]} \n {init_transformationmatrix_dh(2)[2]} \n {init_transformationmatrix_dh(2)[3]}')

    #A = [[1,2,3],[4,1,3],[5,2,2]]
    #B = [[3,2,1],[2,4,3],[4,2,3]]
    #print(np.dot(np.dot(A,B),A)) #A*B*A
    #print(init_chain_transformationmatrix(2))

    #print(f'g_T_id = {init_chain_transformationmatrix(3)}')

    entire_T = init_entire_transformationmatrices()
    print(f'entire_T = {entire_T}')
    A = entire_T[0]
    print(f'A: {A}')

    #ursprung_punkt_show = np.array([[0], [0], [0], [1]])





    # punkt_P = np.array([[0], [0], [0]])
    # print(f'punkt_P: {punkt_P}')
    # punkt_P_hom = np.append(punkt_P, [1])
    # print(f'punkt_P_hom: {punkt_P_hom}')
    # punkt_P_calc = [punkt_P_hom[0], punkt_P_hom[1], punkt_P_hom[2]]
    # print(f'punkt_P_calc: {punkt_P_calc}')
    #print([punkt_P[0, 0], punkt_P[1, 0], punkt_P[1, 0]])


    # A_show = A[:3, :3]
    # print(f'A_show:\n {A_show}')


    punkt_P_hom = np.array([[0], [0], [0], [1]])
    print(f'punkt_P:\n {punkt_P_hom}')
    #punkt_P_show = punkt_P_hom[:3, :1]
    #print(f'punkt_P_show:\n {punkt_P_show}')




    fig = plt.figure()
    #plt.rcParams['figure.figsize']=(8,6)
    ax=plt.axes(projection='3d')
    #ax.scatter3D(1,2,3)
    #plt.plot([1, 2, 3], [1, 2, 3], 'go-', label='line 1', linewidth=2)
    #plt.plot([punkt_P[0, 0], punkt_P[1, 0], punkt_P[1, 0]], [1, 2, 3], 'go-', label='line 1', linewidth=2)
    #plt.plot(punkt_P_show, [1, 2, 3], 'go-', label='line 1', linewidth=2)
    plt.show()

