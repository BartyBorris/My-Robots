import matplotlib.pyplot as plt
import numpy as np

backLegSensorValues = np.load("data/backLegSensorValues.npy")
frontLegSensorValues = np.load("data/frontLegSensorValues.npy")


plt.plot(backLegSensorValues, label='Back Leg', linewidth='2')
plt.plot(frontLegSensorValues, label='Front Leg')

plt.legend()

plt.show()