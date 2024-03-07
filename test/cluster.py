import time
import numpy as np
import matplotlib.pyplot as plt


def distance(loc1, loc2):
    return np.sqrt((loc1[0] - loc2[0])**2 + (loc1[1] - loc2[1])**2)


def group_locations_naive(locations, threshold):
    groups = []
    visited = set()

    for i in range(len(locations)):
        if i not in visited:
            group = [locations[i]]
            visited.add(i)

            for j in range(i + 1, len(locations)):
                if j not in visited and distance(locations[i], locations[j]) <= threshold:
                    group.append(locations[j])
                    visited.add(j)

            groups.append(group)

    return groups


def group_locations_optimized(locations, threshold):
    locations = np.array(locations)
    n = len(locations)
    visited = np.zeros(n, dtype=bool)
    groups = []

    for i in range(n):
        if not visited[i]:
            group = [tuple(locations[i])]
            visited[i] = True

            for j in range(i + 1, n):
                if not visited[j] and distance(locations[i], locations[j]) <= threshold:
                    group.append(tuple(locations[j]))
                    visited[j] = True

            groups.append(group)

    return groups


# Test data
test_data = [
    np.random.rand(50, 2) * 100,
    np.random.rand(50, 2) * 100,
    np.random.rand(50, 2) * 100
]

threshold = 5

# Execute and time the naive function
naive_times = []
for locations in test_data:
    start_time = time.time()
    naive_result = group_locations_naive(locations.tolist(), threshold)
    end_time = time.time()
    naive_times.append(end_time - start_time)

# Execute and time the optimized function
optimized_times = []
for locations in test_data:
    start_time = time.time()
    optimized_result = group_locations_optimized(locations, threshold)
    end_time = time.time()
    optimized_times.append(end_time - start_time)

# Print the execution times
print("Naive Function Execution Times:")
for i, t in enumerate(naive_times):
    print(f"Test {i+1}: {t:.6f} seconds")

print("\nOptimized Function Execution Times:")
for i, t in enumerate(optimized_times):
    print(f"Test {i+1}: {t:.6f} seconds")

# Plot the execution times
plt.figure(figsize=(8, 6))
plt.plot(range(1, 4), naive_times, marker='o', label='Naive Function')
plt.plot(range(1, 4), optimized_times, marker='o', label='Optimized Function')
plt.xlabel('Test Case')
plt.ylabel('Execution Time (seconds)')
plt.title('Execution Time Comparison')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Plot the cluster results
colors = ['red', 'green', 'blue', 'orange', 'purple',
          'brown', 'pink', 'gray', 'olive', 'cyan']

for i, locations in enumerate(test_data):
    plt.figure(figsize=(8, 6))
    plt.title(f'Cluster Results - Test Case {i+1}')

    for j, group in enumerate(optimized_result):
        group = np.array(group)
        plt.scatter(group[:, 0], group[:, 1], color=colors[j %
                    len(colors)], label=f'Group {j+1}')

    plt.xlabel('X')
    plt.ylabel('Y')
    # plt.legend()
    plt.grid(True)
    plt.tight_layout()

plt.show()
