import numpy as np
import math


class Robot:
    def __init__(self, a):
        self.A = a





class Joint:
    def __init__(self, angle, length, offset, twist):
        self.Angle = angle
        self.Length = length
        self.Offset = offset
        self.Twist = twist

    def init_transformation_matrix_dh(self):
        #print("Winkel, Beispiel: ", self.Angle)
        transMat = [[math.cos(self.Angle), -math.sin(self.Angle) * math.cos(self.Twist), math.sin(self.Angle) * math.sin(self.Twist), self.Length * math.cos(self.Angle)],
                    [math.sin(self.Angle), math.cos(self.Angle) * math.cos(self.Twist), -math.cos(self.Angle) * math.sin(self.Twist), self.Length * math.sin(self.Angle)],
                    [0, math.sin(self.Twist), math.cos(self.Twist), self.Offset],
                    [0, 0, 0, 1]]
        print(transMat)


