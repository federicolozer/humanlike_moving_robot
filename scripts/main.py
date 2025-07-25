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
import os
import sys

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
    
    client_socket.close()

    return response



def controller_client(t_arm, q_arm, t_gripper, q_gripper, ttype, traj):
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

    client_socket.sendto(traj.encode(), ('localhost', 8081))
    
    client_socket.close()



def endTransmission(port):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('localhost', port))

    client_socket.send(b"0")

    client_socket.close()



def optMove(q_array_list, q_actual_array):
    err = nan
    
    if not q_array_list == []:
        for array in q_array_list:
            n_err = np.dot((array[2]-q_actual_array[2]), (array[2]-q_actual_array[2]))

            if n_err-err < 0 or np.isnan(n_err-err):
                q_array = list(array)
                err = n_err
    else:
        q_array = []

    return q_array



def dijkstra(sol_array):
    print("-------dijkstra-----------------")
    q_array = []
    dist = None
    jnt = [0, 1, 2, 3, 4, 5, 6]
    doOnce = True
    for start in sol_array[0]:
        path = 0
        q_arrays = [start]
        for sol in sol_array[1:]:
            dst = None
            mn = sum([x[jnt] for x in q_arrays])/len(q_arrays)
            for node in sol: 
                ndst = sum(abs(mn-node[jnt]))
                if dst == None or ndst < dst:
                    dst = ndst
                    pnt = node
            q_arrays.append(pnt)
            path += dst


        print("---------------------")
        print(path)


        if dist == None or path < dist:
            dist = path
            q_array = q_arrays

    print("---------------------")
    print("Distance = ", dist)

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
    q7_array = []
    q7_real_array = []
    q_actual_array = np.array([0, -0.785398163397, 0, -2.3561944899, 0, 1.57079632679, 0.785398163397])
    #q_actual_array = controller.readJointStates()

    try:
        os.mkdir(f'{pack_path}/data/results/{traj}')
    except:
        pass
    
    try:
        with open(f"{pack_path}/data/dataset/{traj}.csv") as file:
            doOnce = True
            file1 = open(f'{pack_path}/data/results/{traj}/q7.csv', 'w')
            file2 = open(f'{pack_path}/data/results/{traj}/O_EE.csv', 'w')

            for line in csv.reader(file):
                if doOnce:
                    doOnce = False
                    continue

                q7_real_array.append(float(line[7]))
                file1.write(f"{float(line[7])}\n")
                file2.write(f"{float(line[4])}, {float(line[5])}, {float(line[6])}\n")

            file1.close()
            file2.close()
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
        file3 = open(f'{pack_path}/data/results/{traj}/q7_NN.csv', 'w')
        file4 = open(f'{pack_path}/data/results/{traj}/t.csv', 'w')
        for waypoint in trajectory["waypoints"]:
            quater = np.array([float(waypoint["Qx"]), float(waypoint["Qy"]), float(waypoint["Qz"]), float(waypoint["Qw"])])
            O_EE = np.array([float(waypoint["x"]), float(waypoint["y"]), float(waypoint["z"])])

            inputData = np.matrix(np.concatenate((quater, O_EE), axis=0))
            q7 = float(nn.neuralNetwork(model, inputData)[0])

            inputData_array.append(deepcopy(inputData))
            q7_array.append(deepcopy(q7))
            file3.write(f"{q7}\n")

            t_array.append(waypoint["t"])
            file4.write(f"{t_array[-1]}\n")

        tn = time.time()
        file3.close()
        file4.close()
        NN_time = deepcopy(tn-t0)
        print(f"Elapsed time for having a solution from NN: {(NN_time):>4f} s")
        print("---------------------------------------------------------------")

        # Savitzky-Golay filter -----------------------------------------------------------------

        q7_array = savgol_filter(q7_array, window_length=int(0.2*len(q7_array)), polyorder=3)

        # Inverse kinematics -----------------------------------------------------------------

        print("\n===============================================================")
        print("\tInverse kinematics")
        print("===============================================================")
        t0 = time.time()
        cnt = 0
        sol_array = []
        for i in range(len(inputData_array)):
            inputData = inputData_array[i]
            q7 = q7_array[i]
            
            data = [float(inputData[0, 0]), float(inputData[0, 1]), float(inputData[0, 2]), float(inputData[0, 3]), float(inputData[0, 4]), float(inputData[0, 5]), float(inputData[0, 6]), q7, float(mode), float(dispFrame)]
            res = IK_fromQuater_client(data)

            if not res == []:
                sol_array.append(res)
                cnt += 1

            #q_array = optMove(response, q_actual_array)

            #if not len(q_array) == 0:
            #    t_arm.append(t_array[i]*sd_rate)
            #    q_arm.append(q_array)
            #    q_actual_array = q_array
            #    cnt += 1
            #else:
            #    pass
            #    #t.append(None)
            #    #q.append(None)
        
        tn = time.time()
        IK_time = deepcopy(tn-t0)
        print(f"Elapsed time for having a solution from IK client: {(IK_time):>4f} s")
        print("---------------------------------------------------------------")

        # Test evaluation ----------------------------------------------------------------

        diff = 0
        for i in range(len(q7_array)):
            comp = (q7_array[i]-q7_real_array[i])**2
            diff += comp

        diff /= len(q7_array)
        rmse = np.sqrt(diff)
        error = rmse/(2*2.8973)*100

        print(f"RMSE: {(rmse):>0.4f} rad - {(error):>0.1f}%")

        # Trajectory planning ----------------------------------------------------------------

        print("\n===============================================================")
        print("\tTrajectory planning")
        print("===============================================================")
        print(f"Solutions found: {cnt}/{len(trajectory['waypoints'])}")
        print("---------------------------------------------------------------")

        q_array = dijkstra(sol_array)

        for i in range(len(q_array)):
                t_arm.append(t_array[i]*sd_rate)
                q_arm.append(q_array[i])

        controller_client(t_arm, q_arm, t_gripper, q_gripper, ttype, traj)

        name = traj.replace("_", "\_")
        latex = f"\t${name}$ & {rmse:>1.3f} & {error:>1.3f} & {NN_time:>1.3f} & {IK_time:>1.3f} & {cnt}/{len(trajectory['waypoints'])}\\\\   %{traj}\n"

        file = open(f'{pack_path}/data/latex/table.txt', 'a')
        file.write(latex)
        file.close()
    
    return 1








if __name__ == '__main__':
    if len(sys.argv) > 1:
        for traj in sys.argv[1:]:
            main(traj)
    else:
        #while True:
        print("\n===============================================================")
        #traj = input("Type the trajectory to perform or quit to exit:\n")
        traj = "pnp_4bricks_flip"

        #if traj == "quit":
        #    #endTransmission(8080)
        #    break

        main(traj)
