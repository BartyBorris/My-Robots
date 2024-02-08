import constants as c
import numpy as np
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:
        
    def __init__(self, jointName):
        self.jointName = jointName

    def Prepare_To_Act(self):

        if self.jointName == b"Torso_BackLeg":
            self.amplitude = c.backLeg_amplitude
            self.frequency = c.backLeg_frequency
            self.phaseOffset = c.backLeg_phaseOffset
        
        if self.jointName == b"Torso_FrontLeg":
            self.amplitude = c.frontLeg_amplitude
            self.frequency = 20*c.frontLeg_frequency
            self.phaseOffset = c.frontLeg_phaseOffset
        

        self.targetAngles = np.linspace(0, 2*np.pi, 1000)
        for i, value in enumerate(self.targetAngles):
            self.targetAngles[i] = self.amplitude * np.sin(self.frequency * self.targetAngles[i] + self.phaseOffset)

    def Set_Value(self, robotId, i):
        
        pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, 
                                    jointName=self.jointName, 
                                    controlMode=p.POSITION_CONTROL, 
                                    targetPosition=np.sin(self.targetAngles[i]), 
                                    maxForce=500)
