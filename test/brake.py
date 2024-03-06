import math
def compute_brake(speed,distance):
        brake = math.exp(-distance/10)
        return math.sqrt(brake**2)

# Test cases
for i in range(1,30):
    print(compute_brake(20, i))  # 0.1

import matplotlib.pyplot as plt
import numpy as np
x = np.linspace(0, 50, 80)
y = []
for i in x:
    y.append(compute_brake(20, i))
plt.plot(x, y)
plt.show()