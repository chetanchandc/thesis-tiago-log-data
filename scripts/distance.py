#!/usr/bin/env python

import pandas as pd


dataset = pd.read_csv("/home/trayach/tiago_public_ws/src/log_data/log/move_base/5_persons/exp1_m-b_5p_log_2022-7-14-11-9.csv")



distance1 = math.sqrt( ((int(robotPosition.x)-int(pedPosition1.x))**2)+((int(robotPosition.y)-int(pedPosition1.y))**2) )



print (dataset)
