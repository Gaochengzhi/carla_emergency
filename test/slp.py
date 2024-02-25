from scipy.interpolate import splprep, splev
import numpy as np


waypoints = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]

xy_list = np.array(waypoints)
ws = np.linspace(10, 20, len(xy_list))
xy_list = np.column_stack([xy_list, ws])


tck, u = splprep(xy_list.T, s=4)
u_new = np.linspace(u.min(), u.max(), 30)
x_fine, y_fine, s_fine = splev(u_new, tck)
interpolated_waypoints = np.column_stack([x_fine, y_fine]).tolist()

print(interpolated_waypoints[:1])
