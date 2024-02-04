import pybullet as p 
import pybullet_data
import time
import pyrosim.pyrosim as pyrosim
import numpy
import random

# Create the GUI 
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

# create sensor lists
backLegSensorValues = numpy.zeros(1000)
frontLegSensorValues = numpy.zeros(1000)

# create motor angle lists
frontLeg_amplitude = numpy.pi/4
frontLeg_frequency = 13
frontLeg_phaseOffset = 2

backLeg_amplitude = numpy.pi/4
backLeg_frequency = 13
backLeg_phaseOffset = 2

frontLeg_targetAngles = numpy.linspace(0, 2*numpy.pi, 1000)
for i, value in enumerate(frontLeg_targetAngles):
    frontLeg_targetAngles[i] = frontLeg_amplitude * numpy.sin(frontLeg_frequency * frontLeg_targetAngles[i] + frontLeg_phaseOffset)

backLeg_targetAngles = numpy.linspace(0, 2*numpy.pi, 1000)
for i, value in enumerate(backLeg_targetAngles):
    backLeg_targetAngles[i] = backLeg_amplitude * numpy.sin(backLeg_frequency * backLeg_targetAngles[i] + backLeg_phaseOffset)

# loop over simulation
for i in range(1000):
    # step the simulation
    p.stepSimulation()

    # add a sensor value to the sensor lists
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=b"Torso_BackLeg", controlMode=p.POSITION_CONTROL, targetPosition=numpy.sin(backLeg_targetAngles[i]), maxForce=500)

    pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=b"Torso_FrontLeg", controlMode=p.POSITION_CONTROL, targetPosition=numpy.sin(frontLeg_targetAngles[i]), maxForce=500)

    time.sleep(1/60)

numpy.save("data/backLegSensorValues.npy", backLegSensorValues)
numpy.save("data/frontLegSensorValues.npy", frontLegSensorValues)


p.disconnect()
