from datetime import datetime
from typing import final

import utility

import pandas as pd
import numpy as np
import os


x_pos_name = 'robot_pos_x(in meters)'
y_pos_name = 'robot_pos_y(in meters)'
vel_col = 'robot_lin_vel_x(in m/s)'
first_target = np.array([9,9])

threshold = .3

paths = ['./log/human_aware_navigation/2_goals', './log/move_base/2_goals']
results = dict()
for path in paths:
    key = path.split('/')[2]
    results[key] = dict()
    for sub_key  in os.listdir(path):
        exp_path = os.path.join(path,sub_key)
        results[key][sub_key] = dict()
        results[key][sub_key]['col_names'] = list()
        results[key][sub_key]['real_time_requirement_t1'] = list()
        results[key][sub_key]['real_time_requirement_t2'] = list()
        results[key][sub_key]['sim_time_requirement_t1'] = list()
        results[key][sub_key]['sim_time_requirement_t2'] = list()
        results[key][sub_key]['speed_t1'] = list()
        results[key][sub_key]['speed_t2'] = list()
        results[key][sub_key]['acceleration_t1'] = list()
        results[key][sub_key]['acceleration_t2'] = list()
        results[key][sub_key]['min_distance_t1'] = list()
        results[key][sub_key]['min_distance_t2'] = list()
        for experiment in os.listdir(exp_path):
            results[key][sub_key]['col_names'].append(experiment.split('.')[0])
            experiment_csv_path = os.path.join(exp_path,experiment)
            exp_df = pd.read_csv(experiment_csv_path)
            col_names = list(exp_df.columns)
            size = exp_df.shape[0]
            dist = np.inf
            cur_row = 0
            first_target_matrix = []
            second_target_matrix = []
            while True:
                current_pos = np.array(exp_df[[x_pos_name,y_pos_name]].loc[cur_row])
                dist = np.linalg.norm(current_pos-first_target)
                if dist > threshold:
                    row = list(exp_df.loc[cur_row])
                    first_target_matrix.append(row)
                    cur_row += 1
                else:
                    break
            while True:
                current_pos = np.array(exp_df[[x_pos_name,y_pos_name]].loc[cur_row])
                dist = np.linalg.norm(current_pos-first_target)
                if dist > threshold:
                    break
                cur_row+=1
            while True:
                second_target_matrix.append(list(exp_df.loc[cur_row]))
                cur_row += 1
                if cur_row>=size:
                    break

            real_time_req_target_1 = utility.calculate_time_requirement(first_target_matrix,1)
            real_time_req_target_2 = utility.calculate_time_requirement(second_target_matrix,1)
            #print("Time requirement for first target : {} seconds".format(int(real_time_req_target_1)))
            #print("Time requirement for second target : {} seconds".format(int(real_time_req_target_2)))

            sim_time_req_target_1 = utility.calculate_time_requirement(first_target_matrix,0)
            sim_time_req_target_2 = utility.calculate_time_requirement(second_target_matrix,0)
            #print("Time requirement for first target : {} seconds".format(int(sim_time_req_target_1)))
            #print("Time requirement for second target : {} seconds".format(int(sim_time_req_target_2)))

            avg_speed_target_1 = utility.compute_average_speed(first_target_matrix,col_names.index(vel_col))
            avg_speed_target_2 = utility.compute_average_speed(second_target_matrix,col_names.index(vel_col))
            #print("Average speed of robot on first target : {} seconds".format(avg_speed_target_1))
            #print("Average speed of robot on second target : {} seconds".format(avg_speed_target_2))

            acceleration_target_1 = utility.compute_acceleration(first_target_matrix,col_names.index(vel_col),sim_time_req_target_1)
            acceleration_target_2 = utility.compute_acceleration(second_target_matrix,col_names.index(vel_col),sim_time_req_target_1)
            #print("acceleration of robot on first target : {} seconds".format(avg_speed_target_1/sim_time_req_target_1))
            #print("acceleration of robot on second target : {} seconds".format(avg_speed_target_2/sim_time_req_target_2))

            avg_min_distance_target_1 = utility.compute_min_distance(first_target_matrix,col_names,[x_pos_name,y_pos_name])
            avg_min_distance_target_2 = utility.compute_min_distance(second_target_matrix,col_names,[x_pos_name,y_pos_name])
            #print("Average min distance of robot on first target : {}".format(avg_min_distance_target_1))
            #print("Average min distance of robot on second target : {}".format(avg_min_distance_target_2))

            results[key][sub_key]['real_time_requirement_t1'].append(real_time_req_target_1)
            results[key][sub_key]['real_time_requirement_t2'].append(real_time_req_target_2)
            results[key][sub_key]['sim_time_requirement_t1'].append(sim_time_req_target_1)
            results[key][sub_key]['sim_time_requirement_t2'].append(sim_time_req_target_2)
            results[key][sub_key]['speed_t1'].append(avg_speed_target_1)
            results[key][sub_key]['speed_t2'].append(avg_speed_target_2)
            results[key][sub_key]['acceleration_t1'].append(acceleration_target_1)
            results[key][sub_key]['acceleration_t2'].append(acceleration_target_2)
            results[key][sub_key]['min_distance_t1'].append(avg_min_distance_target_1)
            results[key][sub_key]['min_distance_t2'].append(avg_min_distance_target_2)
final_result = dict()
final_result['approach'] = list()
final_result['num_person'] = list()
for approach,n_person_result in results.items():
    for n_person, experiment in n_person_result.items():
        final_result['num_person'].append(n_person)
        final_result['approach'].append(approach)
        for col, result_list in experiment.items():
            if col!='col_names':
                mean = np.mean(result_list)
                std = np.std(result_list)
                avg_col = 'average_'+col
                std_col = 'std_'+col
                if avg_col in final_result.keys():
                    final_result[avg_col].append(mean)
                else:
                    final_result[avg_col] = [mean]
                if std_col in final_result.keys():
                    final_result[std_col].append(std)
                else:
                    final_result[std_col] = [std]
                if col.startswith('acceleration'):
                    rms = np.sqrt(np.mean(np.square(result_list)))
                    max_acc = np.max(result_list)
                    if 'rms_'+col in final_result.keys():
                        final_result['rms_'+col].append(rms)
                    else:
                        final_result['rms_'+col] = [rms]
                    if 'max_'+col in final_result.keys():
                        final_result['max_'+col].append(max_acc)
                    else:
                        final_result['max_'+col] = [max_acc]

output_df = pd.DataFrame.from_dict(final_result)
output_df.to_csv('output2.csv')  


                
                
                
        

        



            