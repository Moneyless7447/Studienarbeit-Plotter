import matplotlib as plt
import numpy as np
import math
from xml.etree import ElementTree as ET

# input: Initialisierungsliste init_list = [[v_x,v_y,v_z,phi_x,phi_y,phi_z,parent, range_v_x_min,range_v_x_max, range_v_y_min,range_v_y_max, range_v_z_min,range_v_z_max, range_phi_x_min,range_phi_x_max,  range_phi_y_min,range_phi_y_max, range_phi_z_min,range_phi_z_max,]
init_list = [[1,0,0,math.pi,0,0,-1, 0, 0, 0, 0, 0, 0, 0, math.pi, 0, 2, 0, 0],
             [1,0,0,math.pi,0,0,0, 0, 0, 0, 0, 0, 0, 0, math.pi, 0, 2, 0, 0],
             [1,0,0,math.pi,0,0,1, 0, 0, 0, 0, 0, 0, 0, math.pi, 0, 2, 0, 0]]

# Inititialisierungsliste backup (für den Reset Button um Neueingabe zu verhindern)
backup_init_list=init_list

def init_degree_of_freedom_list(init_list):
    """
init_degree_of_freedom_list erzeugt eine Liste abhängig aus Eintraegen in der init_list mit relevanten Inhalten (Freiheitsgrade).
Diese Liste ist zur Überprüfung der Range/Reichweite eines Freiheitsgrades und die Zuordnung einer Eingabe zum richtigen bewegen.
Bsp.: Der Nutzer hat ein System mit 2 Gelenken (Rotatorische Gelenke, Drehung um z-Achse möglich) initialisiert:
init_list = [[1,0,0,math.pi,0,0,-1,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -pi/2, pi/4]
                [0,1,0,math.pi,0,0,-1,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -pi/4, pi/3]]
-> degree_of_freedom_list ergibt sich: [[['phi_z',-pi/2, pi/4]][['phi_z',-pi/4, pi/3]]]
Als weitere Eintraege soll der Nutzer nun eine Liste mit Aenderungen angeben können, statt eine komplette Initialisierungsliste
-> aenderungs_liste = [[[+pi/2]],[[0]]] (das erste Gelenk soll um +pi/2 rotiert werden)
Das Programm weiß mithilfe der degree_of_freedom_list, dass der 1. eintrag zum 1. Gelenk gehoert und gleichzeitig,
dass es sich um eine Rotation um die z-Achse handelt. Zusätzlich kann hier die Range überprueft und aktualisiert werden.
Ebenso wird die Symbolik hierrüber deklariert.

    :param init_list:
    :return:

"""
    #liste aus init erzeugen, mit der anzahl innere listen = anzahl  initlist, platzhalterliste erzeugen
    degree_of_freedom_list=[[] for i in range(len(init_list))]
    for i in range(len(init_list)):
        if init_list[i][7] != 0 or init_list[i][8] != 0:
            degree_of_freedom_list[i].append(["v_x",init_list[i][7],init_list[i][8]])
        if init_list[i][9] != 0 or init_list[i][10] != 0:
            degree_of_freedom_list[i].append(["v_y",init_list[i][9],init_list[i][10]])
        if init_list[i][11] != 0 or init_list[i][12] != 0:
            degree_of_freedom_list[i].append(["v_z",init_list[i][11],init_list[i][12]])
        if init_list[i][13] != 0 or init_list[i][14] != 0:
            degree_of_freedom_list[i].append(["phi_x",init_list[i][13],init_list[i][14]])
        if init_list[i][15] != 0 or init_list[i][16] != 0:
            degree_of_freedom_list[i].append(["phi_y",init_list[i][15],init_list[i][16]])
        if init_list[i][17] != 0 or init_list[i][18] != 0:
            degree_of_freedom_list[i].append(["phi_z",init_list[i][17],init_list[i][18]])
    print("degree_of_freedom_list: ", degree_of_freedom_list)
    return


"""TRANSFORMATIONSMATRIZEN AUFSTELLEN
Schritte:
1) Welche Transformationsmatrizen müssen erzeugt werden -> init_liste über die parents (Erkennung kinematischer Ketten)
2) Für jedes Gelenk i muss eine Transformationsmatrix -1 bis i berechnet werden -> Einzeltransformationsmatrizen werden multipliziert
"""


def init_parent_list(init_list):
    parent_list = []
    for i in range(len(init_list)):
        parent_list.append(init_list[i][6])
    print("parent_list: ", parent_list)
    return parent_list


def init_kin_chains(init_list, parent_list):
    """
    Eine Liste aus Listen für Gelenk 1 bis n mit der kinematischen Kette für das jeweilige Gelenk
    bis zum Basiskoordinatensystem.
    Bsp.: Bei einem System einer einfachen kinematischen Kette mit 3 Gelenken, die jeweils mit dem
    Vorgaenger verbunden sind saehe die kin_chain_list wie folgt aus: [[-1][0,-1][1,0,-1]]
                                                                      [[G1][G2  ][G3    ]]
    (-1 ist das Basiskoordinatensystem, die anderen Zahlen beziehen sich auf den Index des Gelenks in der init_list = Parent)
    :return:
    """

    kin_chain_list = [[] for i in range(len(init_list))]
    for n in range(len(kin_chain_list)):
        tmp_parent = n
        while parent_list[tmp_parent] != -1:
            kin_chain_list[n] = [(parent_list[tmp_parent]), *kin_chain_list[n]]
            tmp_parent = parent_list[tmp_parent]

        kin_chain_list[n].append(n)
        kin_chain_list[n] = [-1, *kin_chain_list[n]]
    print("kin_chain_list: ", kin_chain_list)
    return kin_chain_list


def init_transformationlist():
    """
    Erzeugt Liste aus nötigen Transformationsmatrizen, nutzt die kin_chain_list
    (z.B. kin_chain_list:  [[-1], [0, -1], [1, 0, -1]])
    Ziel: Liste aus nötigen Kombinationen von Transformationsmatrizen fuer
    jedes Gelenk vom Basiskoordinatensystem -1 zum Gelenk i: T_-1_i

    [ [T_-1_0] [T_-1_0, T_0_1] [T_-1_]]

    """

    return


def init_transformationmatrix(rotationsmatrix, tranVec):
    """
    Erzeugt eine homogene Transformationsmatrix

    [[a_11, a_12, a_13, a_14],[a_21, a_22, a_23, a_24],[a_31, a_32, a_33, a_34][a_41, a_42, a_43, a_44]]

    :param rotationsmatrix:
    :param tranVec:
    :return:


    Schritte:

    """
    # (4D-)Einheitsmatrix
    transformationsmatrix = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    return


def read_xml(documentname):
    xml_tree = ET.parse(documentname)
    # HIER XML DATEI FORMATIERT AUSLESEN
    root = xml_tree.getroot()    # Element 'roboter'

    list_from_xml = []

    # Tiefe der XML Baumes, entspricht maximaler Laenge der Liste list_from_xml bzw. Anzahl der Gelenke
    xml_tree_depth_max = 0
    for i in root.iter('joint'):
        xml_tree_depth_max += 1
    print(f"xml_tree_depth: {xml_tree_depth_max}")



    len_root = len(root)                    # Beispiel hier: 2
    len_root_l2 = len(root[1])              # Beispiel hier: 4, Joint 1.2 Parameter
    print(f"len_root: {len_root}")
    print(f"len_root_l2: {len_root_l2}")
    # Anstatt root[j] wird eigentlich sowas wie currentlayer[] gebraucht
    for j in range(len_root):
        list_from_xml.append(['1', root[j].find('angle').text, root[j].find('length').text,
                              root[j].find('offset').text, root[j].find('twist').text])
        if len_root >= 2:
            list_from_xml[j][0] = list_from_xml[j][0], *'1'
    #print(ET.tostring(root, encoding='utf8').decode('utf8'))


    #for child in root.iter('joint'):
    #    print(child.tag)

    print(f"init_list_from_xml: {list_from_xml}")

"""
"Info zu Gelenk i" = name, DH_1, DH_2, DH_3, DH_4,
Ziel[0][1][0][0] -> Zu Gelenk 3.1.1 gehören
Ziel[0][1][0][1] -> Zu Gelenk 3.1.2 gehören
Ziel = [
            [
                "Info zu Geklenk 1",
                    [
                        "Info zu Gelenk 2.1",
                            [
                                "Info zu Gelenk 3.1.1"
                            ],
                            [
                                "Info zu Gelenk 3.1.2"
                            ]
                    ],
                    [
                        "Info zu Gelenk 2.2"
                    ]
            ]
            [
                "Zusatzgelenk"
            ]
        ]

        --> [["Info zu Geklenk 1",["Info zu Gelenk 2.1",["Info zu Gelenk 3.1.1"],["Info zu Gelenk 3.1.2"]],["Info zu Gelenk 2.2"]]["Zusatzgelenk"]]
"""


if __name__ == '__main__':
    init_degree_of_freedom_list(init_list)
    #parent_list = init_parent_list(init_list)
    #init_kin_chains(init_list, parent_list)
    #R1 = Joint(math.pi, 14, 14, math.pi/4)
    #R1.init_transformation_matrix_dh()
    read_xml('config1.xml')