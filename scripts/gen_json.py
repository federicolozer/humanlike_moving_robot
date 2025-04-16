#!/usr/bin/env python3
# coding=utf-8
 
import os
import json
import rospkg

pack_path = rospkg.RosPack().get_path("humanlike_moving_robot")
data_path = f"{pack_path}/data/tracking_data"
json_path = f"{pack_path}/data/tracking_data/data.json"



def gen_json():
    data_list = []
    
    for folder in os.walk(data_path):
        for file in folder[2]:
            data_list.append(file)

    with open(json_path, "w") as file:
        data = {}
        for i in range(len(data_list)):
            data[f"Data_{i+1}"] = data_list[i]
        
        json.dump(data, file, indent=4)
    
    return data


