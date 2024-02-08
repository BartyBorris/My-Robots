import numpy as np
import pyrosim.pyrosim as pyrosim

class SENSOR:
        
        def __init__(self, linkName):
            
            self.linkName = linkName
            self.values = np.zeros(1000)

        def Get_Value(self, t):
            
            # frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
            self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
            #print(self.values[t])

        def Save_Values(self, fileName):
            np.save(fileName, self.values)