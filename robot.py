from motor import MOTOR
from sensor import SENSOR
import pybullet as p 
import pyrosim.pyrosim as pyrosim
import numpy as np

class  ROBOT:
        
        def __init__(self):
            
            self.motor = {}
            self.robotId = p.loadURDF("body.urdf")
        
        def Prepare_To_Sense(self):
            
            self.sensors = {}
            
            for linkName in pyrosim.linkNamesToIndices:
                self.sensors[linkName] = SENSOR(linkName)

        def Prepare_To_Act(self):
             
            self.motors = {}
             
            for i, jointName in enumerate(pyrosim.jointNamesToIndices):
                self.motors[jointName] = MOTOR(jointName)
                self.motors[jointName].Prepare_To_Act()
            self.motors[jointName].frequency = 1
                

        def Sense(self, t):
                      
            for sensor in self.sensors:
                self.sensors[sensor].Get_Value(t)

        def Act(self, robotId, i):
             
             for motor in self.motors:
                  self.motors[motor].Set_Value(robotId, i)




            