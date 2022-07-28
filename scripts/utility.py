from datetime import datetime

import pandas as pd
import numpy as np


def calculate_time_requirement(matrix,col):
    start_ts = datetime.fromtimestamp(int(matrix[0][col]))
    end_ts = datetime.fromtimestamp(int(matrix[-1][col]))
    return (end_ts-start_ts).total_seconds()

def compute_average_speed(matrix,col):
    total_speed = 0
    for row in matrix:
        total_speed += row[col]
    return total_speed/len(matrix)
    
def compute_acceleration(matrix,col,time):
    start_vel = matrix[0][col]
    end_vel = matrix[-1][col]
    return (end_vel-start_vel)/time

def compute_min_distance(matrix,col_names,robot_cols):
    min_distances = []
    person_data = {}
    for col_name in col_names:
        if col_name.startswith('person'):
            name_split = col_name.split('_')
            pos_list = []
            if name_split[2].startswith('x'):
                person_data[name_split[0]] = []
                person_data[name_split[0]].append(col_name)
            elif name_split[2].startswith('y'):
                person_data[name_split[0]].append(col_name)
    robot_x_idx = col_names.index(robot_cols[0])
    robot_y_idx = col_names.index(robot_cols[1])
    
    for row in matrix:
        robot_x = row[robot_x_idx]
        robot_y = row[robot_y_idx]
        min_distance = np.inf
        for person_cols in person_data.values():
            person_x_idx = col_names.index(person_cols[0])
            person_y_idx = col_names.index(person_cols[1])
            person_x = row[person_x_idx]
            person_y = row[person_y_idx]

            dist = np.linalg.norm(np.array([robot_x,robot_y])-np.array([person_x,person_y]))
            if dist < min_distance:
                min_distance = dist

        min_distances.append(min_distance)
    return np.sum(min_distances)/len(min_distances)