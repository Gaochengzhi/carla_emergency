import math
import matplotlib.pyplot as plt

def compute_distance2D(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def sigmoid(x):
    return 1 / (1 + math.exp(-x))

def interpolate_points(start, end, spacing=3):
    distance = compute_distance2D(start, end)
    num_points = max(int(distance / spacing), 2)

    x_diff = end[0] - start[0]
    y_diff = end[1] - start[1]

    return [(start[0] + x_diff * sigmoid(i / (num_points - 1) * 6 - 3),
             start[1] + y_diff * sigmoid(i / (num_points - 1) * 6 - 3))
            for i in range(num_points)]

# Example usage
start_point = (0, 0)
end_point = (10, 10)
spacing = 1

interpolated_points = interpolate_points(start_point, end_point, spacing)

# Extract x and y coordinates for plotting
x_coords = [point[0] for point in interpolated_points]
y_coords = [point[1] for point in interpolated_points]

# Plot the interpolated points
plt.figure(figsize=(8, 8))
plt.plot(x_coords, y_coords, 'o-', markersize=5)
plt.plot(start_point[0], start_point[1], 'ro', markersize=10, label='Start')
plt.plot(end_point[0], end_point[1], 'go', markersize=10, label='End')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Interpolated Points (S-Shaped Curve)')
plt.legend()
plt.grid(True)
plt.show()
