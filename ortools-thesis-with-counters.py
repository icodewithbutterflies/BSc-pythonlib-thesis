# -*- coding: utf-8 -*-
"""
Created on Sun May 19 15:18:45 2024

@author: Julia
"""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import random
import time
import psutil



# Constants
N = 150
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

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = distance_matrix
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data

def print_solution(manager, routing, solution, distance_matrix):
    """Prints solution on console and calculates the total distance."""
    index = routing.Start(0)
    plan_output = "Route for vehicle 0:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += distance_matrix[manager.IndexToNode(previous_index)][manager.IndexToNode(index)]
    plan_output += f" {manager.IndexToNode(index)}\n"
    # Add distance back to the depot
    route_distance += distance_matrix[manager.IndexToNode(index)][manager.IndexToNode(routing.Start(0))]
    plan_output += f"Route distance: {route_distance} m\n"
    print(plan_output)
    return route_distance

def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    start_time = time.time()
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    end_time = time.time()
    total_time = end_time - start_time

    # Print solution on console.
    if solution:
        total_distance = print_solution(manager, routing, solution, data["distance_matrix"])
        
    print("Time passed is: ", total_time)
    print("CPU usage is: ", psutil.cpu_percent(total_time))
    print("RAM usage % is", psutil.virtual_memory()[2])
    print("RAM usage is", psutil.virtual_memory()[3]/1000000000)
    

if __name__ == "__main__":
    main()
