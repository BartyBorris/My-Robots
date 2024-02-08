from world import WORLD
from robot import ROBOT
import pybullet as p 
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time

class   SIMULATION:
    
    def __init__(self):
        
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        self.world = WORLD()
        self.robot = ROBOT()
        pyrosim.Prepare_To_Simulate(self.robot.robotId)
        self.robot.Prepare_To_Sense()
        self.robot.Prepare_To_Act()
    

    def __del__(self):
            
            p.disconnect()

    def run(self):
        
        for i in range(100):
            
            # step the simulation
            p.stepSimulation()
            
            self.robot.Sense(i)

            self.robot.Act(self.robot.robotId, i)

            # # add a sensor value to the sensor lists
            # backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
            # frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

            # pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=b"Torso_BackLeg", controlMode=p.POSITION_CONTROL, targetPosition=numpy.sin(backLeg_targetAngles[i]), maxForce=500)

            # pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=b"Torso_FrontLeg", controlMode=p.POSITION_CONTROL, targetPosition=numpy.sin(frontLeg_targetAngles[i]), maxForce=500)

            time.sleep(1/60)
            