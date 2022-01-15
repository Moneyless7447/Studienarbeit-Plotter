import matplotlib.pyplot as plt
import numpy as np
import math
from robot import *
import json
from mpl_toolkits.mplot3d import Axes3D



################################################################
# Json Baumstruktur zu Dictonary umstrukturieren
"""
Geg.: {'robot': [{'angle': 'Winkel', 'length': '1', 'offset': '2', 'twist': '3', 'children': [{'angle': 'a', 'length': 'b', 'offset': 'd', 'twist': 'd'}]}]}
Ziel: joints = {1: {'name': '1', 'parent': '0', 'angle': 'np.pi/4', 'length': '1', 'offset': '2', 'twist': '0'},
                2: {'name': '2', 'parent': '1', 'angle': '0', 'length': '2', 'offset': '0', 'twist': '0'}}
-----------------------------------------------------------------------------------------------------------------------
Geg.: 
{'robot': [{'angle': 'np.pi/2', 'length': '1.2', 'offset': '1.3', 'twist': '0', 'children':          (1)
[{'angle': '0', 'length': '2.3', 'offset': '1', 'twist': 'np.pi', 'children':                        (3)
[{'angle': '0', 'length': '1', 'offset': '3', 'twist': 'np.pi/4'},                                   (5)
{'angle': '-np.pi/2', 'length': '0.5', 'offset': '2', 'twist': '0'}]},                               (6)
{'angle': 'np.pi/4', 'length': '1.5', 'offset': '0.2', 'twist': '-np.pi/4'}]},                       (4)
{'angle': 'np.pi', 'length': '2.4', 'offset': '3', 'twist': '0'}]}                                   (2)

Ziel: joints = {1: {'name2': '1', 'parent': '0', 'angle': 'np.pi/2', 'length': '1.2', 'offset': '1.3', 'twist': 'np.pi'},
                2: {'name2': '2', 'parent': '0', 'angle': 'np.pi', 'length': '2.4', 'offset': '3', 'twist': '0'},
                3: {'name2': '1.1', 'parent': '1', 'angle': '0', 'length': '2.3', 'offset': '1', 'twist': 'np.pi'},
                4: {'name2': '1.2', 'parent': '1', 'angle': 'np.pi/4', 'length': '1.5', 'offset': '0.2', 'twist': '-np.pi/4'},
                5: {'name2': '1.1.1', 'parent': '3', 'angle': '0', 'length': '1', 'offset': '3', 'twist': 'np.pi/4'},
                6: {'name2': '1.1.2', 'parent': '3', 'angle': '-np.pi/2', 'length': '0.5', 'offset': '2', 'twist': '0'}}

-----------------------------------------------------------------------------------------------------------------------
"""

#"angle": "1.57078",

def restructure_dic():
    with open("./example_robot_dh.json") as data:
        test_data = json.load(data)


    def build_tree(data_list: list):
        def _build_tree(data: dict, result: dict = dict(), key: str = "1"):
            if 'children' not in data:
                result[key] = data
                return
            else:
                result[key] = {"angle": data["angle"], "length": data["length"], "offset": data["offset"],
                               "twist": data["twist"], "name": data["name"], "type": data["type"]}
                for index, value in enumerate(data['children']):
                    new_key = f"{key}.{index + 1}"
                    _build_tree(data=value, result=result, key=new_key)
                return

        result = dict()
        for index, value in enumerate(data_list):
            _build_tree(data=value, result=result, key=f"{index + 1}")
        return result


    robot = test_data['robot']
    joints = build_tree(robot)
    print(joints)
    return joints

joints=restructure_dic()
# print(joints["1.1"].get("length"))

# joints = {1: {'name2': '1', 'parent': 0, 'angle': np.pi/2, 'length': 1.2, 'offset': 1.3, 'twist': 0},
#           2: {'name2': '1.1', 'parent': 1, 'angle': 0, 'length': 2.3, 'offset': 1, 'twist': np.pi},
#           3: {'name2': '2', 'parent': 0, 'angle': np.pi, 'length': 2.4, 'offset': 3, 'twist': 0},
#           4: {'name2': '1.2', 'parent': 1, 'angle': np.pi/4, 'length': 1.5, 'offset': 0.2, 'twist': -np.pi/4},
#           5: {'name2': '1.1.1', 'parent': 2, 'angle': 0, 'length': 1, 'offset': 3, 'twist': np.pi/4},
#           6: {'name2': '1.1.2', 'parent': 2, 'angle': -np.pi/2, 'length': 0.5, 'offset': 2, 'twist': 0}}


# joints = {1: {'name': 'Alpha1-Gelenk', 'type': "rotation", 'angle': np.pi/2, 'length': 1.2, 'offset': 1.3, 'twist': 0},
#           1.1: {'name': 'Beta1-Gelenk',"type": "rotation", 'angle': 0, 'length': 2.3, 'offset': 1, 'twist': np.pi},
#           2: {, 'angle': np.pi, 'length': 2.4, 'offset': 3, 'twist': 0},
#           1.2: {'name2': '1.2', 'parent': 1, 'angle': np.pi/4, 'length': 1.5, 'offset': 0.2, 'twist': -np.pi/4},
#           1.1.1: {'name2': '1.1.1', 'parent': 2, 'angle': 0, 'length': 1, 'offset': 3, 'twist': np.pi/4},
#           1.1.2: {'name2': '1.1.2', 'parent': 2, 'angle': -np.pi/2, 'length': 0.5, 'offset': 2, 'twist': 0}}





# Liste der nötigen Transformationsmatrizen, um vom Ursprung zu jedem Gelenk transformieren zu können
def init_kin_chains():
    """
    Eine Liste aus Listen für Gelenk 1 bis n mit der kinematischen Kette für das jeweilige Gelenk
    bis zum Basiskoordinatensystem.
    Ziel, zu Beispiel B, Index in aeusserer Liste entspricht ID in Dictionary (1-6):
    kin_chain_list = [[0], [0,1], [0,2], [0,1,3], [0,1,4], [0,1,3,5], [0,1,3,6]]
    kin_chain_list = [['0'], ['0','1'], ['0','2'], ['0','1','1.1'], ['0','1','1.2'], ['0','1','1.1','1.1.1'], ['0','1','1.1','1.1.2']]
    """

    # kin_chain_list = [[i] for i in range(len(joints)+1)]
    kin_chain_list = [[element] for element in joints]
    kin_chain_list.insert(0, ['0'])

    for n in range(1,len(kin_chain_list)):
        for zeichen in range(len(kin_chain_list[n][-1])-1,-1,-1):
            if kin_chain_list[n][-1][zeichen] == '.':
                kin_chain_list[n].insert(0,kin_chain_list[n][-1][0:zeichen])

    for i in range(1, len(kin_chain_list)):
        kin_chain_list[i].insert(0,'0')

    #[['0'], ['1'], ['1.1'], ['1.1.1'], ['1.1.2'], ['1.2'], ['2']]
    #[['0'], ['1'], ['1','1.1'], ['1.1.1'], ['1.1.2'], ['1.2'], ['2']]

    #print("kin_chain_list: ", kin_chain_list)

    return kin_chain_list



# Gibt eine Transformationsmatrix nach DH Konvention für T_id-1_id aus Daten von Dictionary joints zurueck
def init_transformationmatrix_dh(id):
    #T_id = DH Transformationsmatrix T_id-1_id
    T_id = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]]
    T_id[0][0] = math.cos(float(joints[id].get('angle')))
    T_id[0][1] = -math.sin(float(joints[id].get('angle')))*math.cos(float(joints[id].get('twist')))
    T_id[0][2] = math.sin(float(joints[id].get('angle')))*math.sin(float(joints[id].get('twist')))
    T_id[0][3] = float(joints[id].get('length'))*math.cos(float(joints[id].get('angle')))
    T_id[1][0] = math.sin(float(joints[id].get('angle')))
    T_id[1][1] = math.cos(float(joints[id].get('angle'))) * math.cos(float(joints[id].get('twist')))
    T_id[1][2] = -math.cos(float(joints[id].get('angle'))) * math.sin(float(joints[id].get('twist')))
    T_id[1][3] = float(joints[id].get('length')) * math.sin(float(joints[id].get('angle')))
    T_id[2][1] = math.sin(float(joints[id].get('twist')))
    T_id[2][2] = math.cos(float(joints[id].get('twist')))
    T_id[2][3] = float(joints[id].get('offset'))

    return T_id

# Gibt eine Gesamtransformationsmatrix_0_Gelenk für EIN Gelenk aus der kin_chain_list zurueck
def init_chain_transformationmatrix(kin_chain_list, index):
    #Segment aus kin_chain_list abhaengig von ID
    piece_kin_chain_list=kin_chain_list[index]
    g_T_id = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    for i in range(1, len(piece_kin_chain_list)):
            g_T_id = np.dot(g_T_id, init_transformationmatrix_dh(piece_kin_chain_list[i]))

    return g_T_id

# Gibt eine Liste aus Gesamtransformationsmatrizen für alle Gelenke zurueck
def init_entire_transformationmatrices():
    #Ziel: [[MatrixG0], [MatrixG1], [MatrixG2], ...]
    #id_list = [['0'], ['1'], ['1.1'], ['1.1.1'], ['1.1.2'], ['1.2'], ['2']]
    id_list = [[element] for element in joints]
    id_list.insert(0, ['0'])
    #entire_T_list = [init_chain_transformationmatrix(i, kin_chain_list) for i in range(len(kin_chain_list))]
    entire_T_list = [init_chain_transformationmatrix(kin_chain_list, i) for i in range(len(id_list))]

    return entire_T_list


def points_coord_system(index, entire_T):

    ursprung_punkt_show = np.array([[0], [0], [0], [1]])
    x_axis_show = np.array([[0.7], [0], [0], [1]])
    y_axis_show = np.array([[0], [0.7], [0], [1]])
    z_axis_show = np.array([[0], [0], [0.7], [1]])

    #Transformationsmatrix * Punkt = Punkt
    #print(entire_T[index])
    ursprung_punkt_show = np.dot(entire_T[index], ursprung_punkt_show)
    x_axis_show = np.dot(entire_T[index], x_axis_show)
    y_axis_show = np.dot(entire_T[index], y_axis_show)
    z_axis_show = np.dot(entire_T[index], z_axis_show)

    #print(f'ursprung_punkt_show_hom {ursprung_punkt_show}')
    ursprung_punkt_show = ursprung_punkt_show[:3, :1]
    x_axis_show = x_axis_show[:3, :1]
    y_axis_show = y_axis_show[:3, :1]
    z_axis_show = z_axis_show[:3, :1]
    #print(f'ursprung_punkt_show {ursprung_punkt_show}')



    return [ursprung_punkt_show, x_axis_show, y_axis_show, z_axis_show]

#Zuordnung von ID eines Gelenks zum Index in der Liste
def get_index_id(id):
    #                   |ab hier gehen die IDs los
    # id_list = [['0'], ['1'], ['1.1'], ['1.1.1'], ['1.1.2'], ['1.2'], ['2']]
    id_list = [[element] for element in joints]
    id_list.insert(0, ['0'])
    print(f'id_list{id_list}')
    print(f'id{id}')
    index = 0
    for i in range(len(id_list)):
        if str(id) != str(id_list[i][0]):
            index += 1
        else:
            break
    return index

def get_id_index(index):
    id_list = [[element] for element in joints]
    id = id_list[index][0]
    return id
#print(f'hier   {str(get_id_index(1))}')

if __name__ == '__main__':

    kin_chain_list = init_kin_chains()
    print(f'kin_chain_list: {kin_chain_list}')


    entire_T = init_entire_transformationmatrices()
    print(f'entire_T: {entire_T}')
    fig = plt.figure()
    plt.rcParams['figure.figsize']=(10,20)
    ax = plt.axes(projection='3d')
    #ax = fig.add_subplot(111, projection='3d')


    # Ursprung Koord 0 ######################################################

    index_Ursprung=get_index_id(0) #0

    test_ursprung_0 = points_coord_system(index_Ursprung, entire_T)[0]   # ursprung_punkt_show
    test_ursprung_x_0 = points_coord_system(index_Ursprung, entire_T)[1] # x_axis_show (Punkt für x-Achse)
    test_ursprung_y_0 = points_coord_system(index_Ursprung, entire_T)[2] # y_axis_show
    test_ursprung_z_0 = points_coord_system(index_Ursprung, entire_T)[3] # z_axis_show

    # Plot des Ursprungs und Achsen vom Basiskoordinatensystems
    plt.plot([test_ursprung_0[0, 0]], [test_ursprung_0[1, 0]], [test_ursprung_0[2, 0]], 'wo', label='Ursprung 0',
             linewidth=1)
    plt.plot([test_ursprung_0[0, 0], test_ursprung_x_0[0, 0]*5], [test_ursprung_0[1, 0], test_ursprung_x_0[1, 0]],
             [test_ursprung_0[2, 0], test_ursprung_x_0[2, 0]], 'r-', label='x0',
             linewidth=2)
    plt.plot([test_ursprung_0[0, 0], test_ursprung_y_0[0, 0]], [test_ursprung_0[1, 0], test_ursprung_y_0[1, 0]*5],
             [test_ursprung_0[2, 0], test_ursprung_y_0[2, 0]], 'g-', label='y0',
             linewidth=2)
    plt.plot([test_ursprung_0[0, 0], test_ursprung_z_0[0, 0]], [test_ursprung_0[1, 0], test_ursprung_z_0[1, 0]],
             [test_ursprung_0[2, 0], test_ursprung_z_0[2, 0]*5], 'b-', label='y0',
             linewidth=2)
    #########################################################################
    # Gelenkurspruenge mit Koordinatenachsen plotten#########################

    #print(f'get_id_index(1){get_id_index(1)}')
    #print(f'test: {joints[str(get_id_index(1))].get("name")}')

    for i in range(1, len(entire_T)):
        # Ursprung
        plt.plot([points_coord_system(i, entire_T)[0][0, 0]], [points_coord_system(i, entire_T)[0][1, 0]],
                 [points_coord_system(i, entire_T)[0][2, 0]], 'k,')
        # Abbilden der ID im Plot an den Koordinatenurspruengen
        # ax.text(points_coord_system(i, entire_T)[0][0, 0],
        #         points_coord_system(i, entire_T)[0][1, 0],
        #         points_coord_system(i, entire_T)[0][2, 0], str(get_id_index(i-1)))
        # anstatt ID den angegebenen Namen anzeigen
        ax.text(points_coord_system(i, entire_T)[0][0, 0],
                points_coord_system(i, entire_T)[0][1, 0],
                points_coord_system(i, entire_T)[0][2, 0], str(joints[get_id_index(i - 1)].get("name")))

        # # x-Achse
                                # ursprung_punkt_show[x,0], x_axis_show[x,0]
                                # ursprung_punkt_show[y,0], x_axis_show[y,0]
                                # ursprung_punkt_show[z,0], x_axis_show[z,0]
        plt.plot([points_coord_system(i, entire_T)[0][0, 0], points_coord_system(i, entire_T)[1][0, 0]],
                 [points_coord_system(i, entire_T)[0][1, 0], points_coord_system(i, entire_T)[1][1, 0]],
                 [points_coord_system(i, entire_T)[0][2, 0], points_coord_system(i, entire_T)[1][2, 0]],
                 'r-', label='x', linewidth=0.5)
        #y-Achse
                                # ursprung_punkt_show[x,0], y_axis_show[x,0]
                                # ursprung_punkt_show[y,0], y_axis_show[y,0]
                                # ursprung_punkt_show[z,0], y_axis_show[z,0]
        plt.plot([points_coord_system(i, entire_T)[0][0, 0], points_coord_system(i, entire_T)[2][0, 0]],
                 [points_coord_system(i, entire_T)[0][1, 0], points_coord_system(i, entire_T)[2][1, 0]],
                 [points_coord_system(i, entire_T)[0][2, 0], points_coord_system(i, entire_T)[2][2, 0]],
                 'g-', label='y', linewidth=0.5)
        #z-Achse
                                # ursprung_punkt_show[x,0], z_axis_show[x,0]
                                # ursprung_punkt_show[y,0], z_axis_show[y,0]
                                # ursprung_punkt_show[z,0], z_axis_show[z,0]
        plt.plot([points_coord_system(i, entire_T)[0][0, 0], points_coord_system(i, entire_T)[3][0, 0]],
                 [points_coord_system(i, entire_T)[0][1, 0], points_coord_system(i, entire_T)[3][1, 0]],
                 [points_coord_system(i, entire_T)[0][2, 0], points_coord_system(i, entire_T)[3][2, 0]],
                 'b-', label='z', linewidth=1)

    ########################################################################
    # Verbindungslinien der Koordinatensysteme##############################
    # kin chain Liste zu den jeweiligen Parents
    # kin_chain_list_parent = [[0]]
    # for k in range(1, len(kin_chain_list)):
    #     kin_chain_list_parent.append(kin_chain_list[k][0:-1])


    kin_chain_list_parent = kin_chain_list
    print(kin_chain_list_parent)
    print(f'laenge von index 1: {len(kin_chain_list_parent[1])}')
    for i in range(1, len(kin_chain_list_parent)):
        kin_chain_list_parent[i] = kin_chain_list_parent[i][0:-1]
    print(f'kin_chain_list_parent: {kin_chain_list_parent}')


    # plot einer Verbindungslinie zwischen dem Ursprung eines Gelenks und dem Ursprung des Parentgelenks
    # berechnung der Transformationsmatrix des Parents für Koordinaten
    for i in range(1, len(entire_T)):
        #Ursprung parent, Ursprung selbst -> verbindungslinie
        # plt.plot([np.dot(init_chain_transformationmatrix(i,kin_chain_list_parent), [[0], [0], [0], [1]])[0],points_coord_system(i, entire_T)[0][0, 0]],
        #          [np.dot(init_chain_transformationmatrix(i,kin_chain_list_parent), [[0], [0], [0], [1]])[1],points_coord_system(i, entire_T)[0][1, 0]],
        #          [np.dot(init_chain_transformationmatrix(i,kin_chain_list_parent), [[0], [0], [0], [1]])[2],points_coord_system(i, entire_T)[0][2, 0]],
        #          'k:', linewidth=0.5)
        plt.plot([np.dot(init_chain_transformationmatrix(kin_chain_list_parent,i),[[0], [0], [0], [1]])[0],points_coord_system(i, entire_T)[0][0, 0]],
                [np.dot(init_chain_transformationmatrix(kin_chain_list_parent, i), [[0], [0], [0], [1]])[1],points_coord_system(i, entire_T)[0][1, 0]],
                [np.dot(init_chain_transformationmatrix(kin_chain_list_parent, i), [[0], [0], [0], [1]])[2],points_coord_system(i, entire_T)[0][2, 0]],
                'k:', linewidth=0.5)

    ########################################################################
    # Geometrische Formen im Plotter um die Koordinatensysteme -> je nach Art des Gelenks
    #TODO Unterscheidung der Gelenktypen

    # Rotationsgelenk um Z-Achse (DH-Konvention)
    # u = np.linspace(0, 2 * np.pi, 40)
    # v = np.linspace(0, 2*  np.pi, 40)

    # x = np.outer(np.cos(u), np.sin(v))
    # y = np.outer(np.sin(u), np.sin(v))
    # z = np.outer(np.ones(np.size(u)), np.cos(v))
    scale_cylinder = 0.3
    #              (start,stop,Teilungsfaktor)
    u = np.linspace(0, 2 * np.pi, num=20) #num=step für Kreis
    v = np.linspace(0, 3, num=2) #num=step für Hoehe Cylinder (min 2)
    u2 = np.linspace(0, 2*np.pi, 40)
    v2 = np.linspace(0, 2*np.pi, 40)

    #                     , np.sin(Durchmesser)
    x = np.outer(np.cos(u), np.sin(scale_cylinder))
    #                     , np.sin(Durchmesser)
    y = np.outer(np.sin(u), np.sin(scale_cylinder))
    z = np.outer(np.ones(np.size(u)), np.cos(v))

    x1 = np.outer(np.cos(u2), np.sin(v2))
    y1 = np.outer(np.sin(u2), np.sin(v2))
    z1 = np.outer(np.zeros(np.size(1)), np.cos(v2))


    # x1 = np.outer(np.cos(u), np.sin(scale_cylinder))
    # y1 = np.outer(np.sin(u), np.sin(scale_cylinder))
    # z1 = np.outer(np.ones(np.size(u)), np.cos(v2))


    #Plot the surface
    ax.plot_surface(x, y, z, color='k')
    #ax.plot_surface(x1,y1,z1, color='b')
    ###




    ########################################################################
    #TODO Variable min und max
    ax.set_xlim3d([-7, 7])
    ax.set_ylim3d([-7, 7])
    ax.set_zlim3d([-7, 7])

    plt.show()

