# -*- coding: utf-8 -*-
"""
Created on Sun May 19 14:40:30 2024

@author: Julia
"""

import random
import numpy as np
from scipy.optimize import linear_sum_assignment
import time
import psutil

# Constants
N = 1000
aisle_width_x = [1.2, 1.5, 1.5, 2.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 2.5, 1.5, 1.5, 1.5, 1.2]
aisle_length_z = [3, 3, 3]
rack_x = 0.5
rack_z = 0.8
list_for_pickup = []

def rack_to_coord_x(rack_random_number):
    aisle_num = rack_random_number // 48
    coord_x = aisle_num * 2 * rack_x
    Count = 0
    while Count <= aisle_num:
        coord_x += aisle_width_x[Count]
        Count += 1
    if rack_random_number % 2 == 0:
        coord_x += 2 * rack_x
    return coord_x

def rack_to_coord_z(rack_random_number):
    coord_z = 0
    if rack_random_number % 48 < 22:
        coord_z = round(aisle_length_z[0] + (rack_random_number % 48) // 2 * rack_z + rack_z * 0.5, 2)
    else:
        coord_z = round(aisle_length_z[0] + aisle_length_z[1] + (rack_random_number % 48) // 2 * rack_z + rack_z * 0.5, 2)
    return coord_z

def minimal_path(point_1_x, point_1_z, point_2_x, point_2_z):
    T_z = aisle_length_z[0] / 2
    B_z = aisle_length_z[0] + aisle_length_z[1] + aisle_length_z[2] / 2 + 24 * rack_z
    min_path = min(
        abs(point_1_z - point_2_z) + abs(point_1_x - point_2_x),
        abs(point_1_z - B_z) + abs(point_1_x - point_2_x) + abs(point_2_z - B_z),
        abs(point_1_z - T_z) + abs(point_1_x - point_2_x) + abs(point_2_z - T_z)
    )
    return min_path

# Generate random pickup list
for _ in range(N):
    list_for_pickup.append(random.randint(0, 719))

print("List for pickup:", list_for_pickup)

# Create distance matrix
distance_matrix = []
for i in range(N):
    row = []
    for j in range(N):
        distance = minimal_path(
            rack_to_coord_x(list_for_pickup[i]), rack_to_coord_z(list_for_pickup[i]),
            rack_to_coord_x(list_for_pickup[j]), rack_to_coord_z(list_for_pickup[j])
        )
        row.append(distance)
    distance_matrix.append(row)

print("Distance matrix:")
for row in distance_matrix:
    print(row)

# Convert distance matrix to numpy array
start_time = time.time()
distance_matrix = np.array(distance_matrix)

# Solve the assignment problem
row_ind, col_ind = linear_sum_assignment(distance_matrix)

# Print the solution
print("Optimal assignment:")
for i in range(len(row_ind)):
    print(f"Pickup point {list_for_pickup[row_ind[i]]} assigned to route position {col_ind[i]}")
total_distance = distance_matrix[row_ind, col_ind].sum()
end_time = time.time()
total_time = end_time - start_time

print("Time passed is: ", total_time)
print("CPU usage is: ", psutil.cpu_percent(total_time))
print("RAM usage % is", psutil.virtual_memory()[2])
print("RAM usage is", psutil.virtual_memory()[3]/1000000000)
# Calculate the total distance of the route
