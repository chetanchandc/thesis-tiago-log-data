from cProfile import label
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

output_folder = 'plot_images'
paths = ['./log/human_aware_navigation/2_goals/5_persons', './log/move_base/2_goals/5_persons']
for path in paths:
    for file in os.listdir(path):
        exp_df = pd.read_csv(os.path.join(path,file))
        robot = []
        person1 = []
        person2 = []
        person3 = []
        person4 = []
        person5 = []
        for idx in range(exp_df.shape[0]):
            robot.append((exp_df['robot_pos_x(in meters)'][idx],exp_df['robot_pos_y(in meters)'][idx]))
            person1.append((exp_df['person1_pos_x(in meters)'][idx],exp_df['person1_pos_y(in meters)'][idx]))
            person2.append((exp_df['person2_pos_x(in meters)'][idx],exp_df['person2_pos_y(in meters)'][idx]))
            person3.append((exp_df['person3_pos_x(in meters)'][idx],exp_df['person4_pos_y(in meters)'][idx]))
            person4.append((exp_df['person4_pos_x(in meters)'][idx],exp_df['person4_pos_y(in meters)'][idx]))
            person5.append((exp_df['person5_pos_x(in meters)'][idx],exp_df['person5_pos_y(in meters)'][idx]))
        robot = np.array(robot)
        person1 = np.array(person1)
        person2 = np.array(person2)
        person3 = np.array(person3)
        person4 = np.array(person4)
        person5 = np.array(person5)
        
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.set_title(file)
        #ax.plot(robot[:,0],robot[:,1],label='Robot')
        ax.scatter(robot[:,0],robot[:,1],s=2,label='Robot')
        ax.plot(person1[:,0],person1[:,1],label='Person 1')
        ax.plot(person2[:,0],person2[:,1],label='Person 2')
        ax.plot(person3[:,0],person3[:,1],label='Person 3')
        ax.plot(person4[:,0],person4[:,1],label='Person 4')
        ax.plot(person5[:,0],person5[:,1],label='Person 5')
        plt.legend()
        plt.show()
        break
    break    
