# -*- coding: utf-8 -*-
"""
Created on Sun May 19 14:40:30 2024

@author: Julia
"""

import random
import numpy as np
import time
import psutil
import pyomo.environ as pyo
from pyomo.opt import SolverFactory

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

# Create Pyomo model
model = pyo.ConcreteModel()

# Sets
model.N = pyo.RangeSet(0, N-1)

# Variables
model.x = pyo.Var(model.N, model.N, within=pyo.Binary)

# Objective
def obj_rule(model):
    return sum(distance_matrix[i, j] * model.x[i, j] for i in model.N for j in model.N)
model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

# Constraints
def row_sum_rule(model, i):
    return sum(model.x[i, j] for j in model.N) == 1
model.row_sum = pyo.Constraint(model.N, rule=row_sum_rule)

def col_sum_rule(model, j):
    return sum(model.x[i, j] for i in model.N) == 1
model.col_sum = pyo.Constraint(model.N, rule=col_sum_rule)

# Solve the problem
solver = SolverFactory('glpk')  # You can use any solver available, e.g., 'cbc', 'gurobi'
result = solver.solve(model, tee=True)

# Extract the results
row_ind = []
col_ind = []
for i in model.N:
    for j in model.N:
        if pyo.value(model.x[i, j]) == 1:
            row_ind.append(i)
            col_ind.append(j)

# Print the solution
print("Optimal assignment:")
for i in range(len(row_ind)):
    print(f"Pickup point {list_for_pickup[row_ind[i]]} assigned to route position {col_ind[i]}")

end_time = time.time()
total_time = end_time - start_time

print("Time passed is: ", total_time)
print("CPU usage is: ", psutil.cpu_percent(total_time))
print("RAM usage % is", psutil.virtual_memory()[2])
print("RAM usage is", psutil.virtual_memory()[3]/1000000000)

