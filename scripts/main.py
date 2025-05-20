#!/usr/bin/env python3
# coding=utf-8

import rospy
import NN_engine as nn
from copy import deepcopy
import numpy as np
from math import pi, nan
import controller
import json
import time
import socket
import yaml
import rospkg
import csv
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import numpy as np



dispFrame = False
ttype = "follow_joint"
sd_rate = 1
t_arm = []
q_arm = []
t_gripper = []
q_gripper = []
inputData_array = []
q7_array = []
q_actual_array = np.array([0, -0.785398163397, 0, -2.3561944899, 0, 1.57079632679, 0.785398163397])
pack_path = rospkg.RosPack().get_path("humanlike_moving_robot")
yaml_path = f"{pack_path}/config/mode.yaml"

model = nn.createModel()



def IK_fromQuater_client(data):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('localhost', 8080))

    client_socket.send(b"1")
    request = np.array(data, dtype=np.double).tobytes()
    client_socket.send(request)

    response = []
    for i in range(4):
        res = np.frombuffer(client_socket.recv(56), dtype=np.double)
        if np.isnan(res).any() == False:
            response.append(res)
        else:
            response.append([])
    
    client_socket.close()

    return response



def controller_client(t_arm, q_arm, t_gripper, q_gripper, ttype):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    data = np.array(t_arm, dtype=np.double).tobytes()
    client_socket.sendto(str(len(data)).encode(), ('localhost', 8081))
    client_socket.sendto(data, ('localhost', 8081))

    data = np.array(q_arm, dtype=np.double).tobytes()
    client_socket.sendto(str(len(data)).encode(), ('localhost', 8081))
    client_socket.sendto(data, ('localhost', 8081))

    data = np.array(t_gripper, dtype=np.double).tobytes()
    client_socket.sendto(str(len(data)).encode(), ('localhost', 8081))
    client_socket.sendto(data, ('localhost', 8081))

    data = np.array(q_gripper, dtype=np.double).tobytes()
    client_socket.sendto(str(len(data)).encode(), ('localhost', 8081))
    client_socket.sendto(data, ('localhost', 8081))

    client_socket.sendto(ttype.encode(), ('localhost', 8081))
    
    client_socket.close()



def endTransmission(port):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('localhost', port))

    client_socket.send(b"0")

    client_socket.close()



def optMove(q_array_list, q_actual_array):
    err = nan
    cnt = 0
    count = 0
    print()
    print("--------------------------")
    print("len = ", len(q_array_list))
    
    if not q_array_list == []:
        q_array = list(q_array_list[1])
        """for array in q_array_list:
            n_err = 0
            cnt += 1

            print("array = ", array)
            
            #n_err = np.dot((array[2]-q_actual_array[2]), (array[2]-q_actual_array[2]))
            
            for i in range(3):
                n_err += np.dot((array[i]-q_actual_array[i]), (array[i]-q_actual_array[i]))

            if n_err-err < 0 or np.isnan(n_err-err):
                q_array = list(array)
                count = deepcopy(cnt)
                err = n_err"""
    else:
        q_array = []
    print("sol = ", count)
    print("q_array = ", q_array)
    return q_array



def sel_mode():
    with open(yaml_path, 'r') as file:
        param = yaml.safe_load(file)["mode"]
        if param == "vert":
            mode = 0
        elif param == "horz":
            mode = 1
        elif param == "ceil":
            mode = 2    

    return mode


 
def main(traj):
    if traj == "quit":
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.sendto(b"0", ('localhost', 8081))
        client_socket.close()
        return -1

    mode = sel_mode()
    t_arm = []
    q_arm = []
    t_gripper = []
    q_gripper = []
    inputData_array = []
    O_EE_array = []
    q7_array = []
    q7_real_array = []
    q_actual_array = np.array([0, -0.785398163397, 0, -2.3561944899, 0, 1.57079632679, 0.785398163397])
    #q_actual_array = controller.readJointStates()
    
    try:
        with open(f"{pack_path}/data/dataset/{traj}.csv") as file:
            doOnce = True
            for line in csv.reader(file):
                if doOnce:
                    doOnce = False
                    continue

                q7_real_array.append(float(line[7]))
    except:
        print("Selected trajectory does not exist..")
        return 0
        
    with open(f"{pack_path}/data/trajectory/{traj}/gripper.json", "r") as file:
        trajectory = json.load(file)

        for waypoint in trajectory["waypoints"]:
            if waypoint["action"] == "close":
                q_array = 0
            elif waypoint["action"] == "open":
                q_array = 1
            t_gripper.append(waypoint["t"]*sd_rate)
            q_gripper.append(q_array)
    
    with open(f"{pack_path}/data/trajectory/{traj}/arm.json", "r") as file:
        trajectory = json.load(file)

        t_array = []

        # Neural network ---------------------------------------------------------------------

        print("\n===============================================================")
        print("\tNeural network")
        print("===============================================================")
        t0 = time.time()
        for waypoint in trajectory["waypoints"]:
            quater = np.array([float(waypoint["Qx"]), float(waypoint["Qy"]), float(waypoint["Qz"]), float(waypoint["Qw"])])
            O_EE = np.array([float(waypoint["x"]), float(waypoint["y"]), float(waypoint["z"])])
            O_EE_array.append(O_EE)

            inputData = np.matrix(np.concatenate((quater, O_EE), axis=0))
            q7 = float(nn.neuralNetwork(model, inputData)[0]) 

            inputData_array.append(deepcopy(inputData))
            q7_array.append(deepcopy(q7))

            t_array.append(waypoint["t"])

        tn = time.time()
        print(f"Elapsed time for having a solution from NN: {(tn-t0):>4f} s")
        print("---------------------------------------------------------------")

        # Savitzky-Golay filter -----------------------------------------------------------------

        x1 = np.array(range(len(q7_array)))
        y1 = np.array(q7_array)

        q7_array = savgol_filter(q7_array, window_length=int(0.1*len(q7_array)), polyorder=3)

        y2 = np.array(O_EE_array)

        # Inverse kinematics -----------------------------------------------------------------

        print("\n===============================================================")
        print("\tInverse kinematics")
        print("===============================================================")
        t0 = time.time()
        cnt = 0
        for i in range(len(inputData_array)):
            inputData = inputData_array[i]
            q7 = q7_array[i]
            
            data = [float(inputData[0, 0]), float(inputData[0, 1]), float(inputData[0, 2]), float(inputData[0, 3]), float(inputData[0, 4]), float(inputData[0, 5]), float(inputData[0, 6]), q7, float(mode), float(dispFrame)]
            response = IK_fromQuater_client(data)

            print("waypoint = ", cnt)
            q_array = optMove(response, q_actual_array)

            if not len(q_array) == 0:
                t_arm.append(t_array[i]*sd_rate)
                q_arm.append(q_array)
                q_actual_array = q_array
                cnt += 1
            else:
                pass
                #t_arm.append(t_array[i]*sd_rate)     #iot ce fa di chescj
                #q_arm.append([None, None, None, None, None, None, None])
        

        x2 = np.array(range(len(q_arm)))
        y4 = np.array(q_arm)[:, 1]


        plt.plot(x1, y1, ".")
        plt.plot(x1, y2, "-")
        plt.plot(x2, y4, "-")
        print("Showing plot")
        plt.show()

        tn = time.time()
        print(f"Elapsed time for having a solution from IK client: {(tn-t0):>4f} s")
        print("---------------------------------------------------------------")

        # Test evaluation ----------------------------------------------------------------

        diff = 0
        for i in range(len(q7_array)):
            comp = (q7_array[i]-q7_real_array[i])**2
            diff += comp

        diff /= len(q7_array)
        rmse = np.sqrt(diff)

        print(f"RMSE: {(rmse):>0.4f} rad - {(rmse/(2*2.8973)*100):>0.1f}%")

        # Trajectory planning ----------------------------------------------------------------

        print("\n===============================================================")
        print("\tTrajectory planning")
        print("===============================================================")
        print(f"Solutions found: {cnt}/{len(trajectory['waypoints'])}")
        print("---------------------------------------------------------------")

        controller_client(t_arm, q_arm, t_gripper, q_gripper, ttype)
        #controller.launch_trajectory(t_arm, q_arm, t_gripper, q_gripper, ttype)
    
    return 1








if __name__ == '__main__':
    while True:
        print("\n===============================================================")
        traj = input("Type the trajectory to perform or quit to exit:\n")

        if traj == "quit":
            #endTransmission(8080)
            break

        main(traj)
