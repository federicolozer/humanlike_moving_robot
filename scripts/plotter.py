#!/usr/bin/env python3
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
import scienceplots
#import rospkg
import sys
import os
from math import floor, ceil

show = True
ygap = 0.4
#pack_path = rospkg.RosPack().get_path("humanlike_moving_robot")
pack_path = "/home/pain/Desktop/humanlike_moving_robot"

def plotter(traj):
    try:
        os.mkdir(f'{pack_path}/data/plots/{traj}')
    except:
        pass

    file1 = open(f'{pack_path}/data/results/{traj}/q7.csv', 'r')
    file2 = open(f'{pack_path}/data/results/{traj}/q7_NN.csv', 'r')
    file3 = open(f'{pack_path}/data/results/{traj}/t.csv', 'r')
    file4 = open(f'{pack_path}/data/results/{traj}/O_EE.csv', 'r')
    file5 = open(f'{pack_path}/data/results/{traj}/O_EE_exp.csv', 'r')
    file6 = open(f'{pack_path}/data/results/{traj}/t_exp.csv', 'r')
    file7 = open(f'{pack_path}/data/results/{traj}/q_exp.csv', 'r')

    q7 = file1.read()
    q7_NN = file2.read()
    t = file3.read()
    O_EE = file4.read()
    O_EE_exp = file5.read()
    t_exp = file6.read()
    q_exp = file7.read()

    file1.close()
    file2.close()
    file3.close()
    file4.close()
    file5.close()
    file6.close()
    file7.close()

    q7 = q7.split()
    for i in range(len(q7)):
        q7[i] = float(q7[i])

    q7_NN = q7_NN.split()
    for i in range(len(q7_NN)):
        q7_NN[i] = float(q7_NN[i])

    t = t.split()
    for i in range(len(t)):
        t[i] = float(t[i])

    O_EE = O_EE.split("\n")
    O_EE.pop(-1)
    x = []
    y = []
    z = []
    for i in range(len(O_EE)):
        tmp = O_EE[i].split(", ")   
        O_EE[i] = [float(tmp[0]), float(tmp[1]), float(tmp[2])]            
        x.append(float(tmp[0]))
        y.append(float(tmp[1]))
        z.append(float(tmp[2]))

    O_EE_exp = O_EE_exp.split("\n")
    O_EE_exp.pop(-1)
    x_exp = []
    y_exp = []
    z_exp = []
    for i in range(len(O_EE_exp)):
        tmp = O_EE_exp[i].split(", ") 
        O_EE_exp[i] = [float(tmp[0]), -float(tmp[1]), 1.033-float(tmp[2])]   
        x_exp.append(float(tmp[0]))
        y_exp.append(-float(tmp[1]))
        z_exp.append(1.033-float(tmp[2]))

    t_exp = t_exp.split()
    for i in range(len(t_exp)):
        t_exp[i] = float(t_exp[i])

    q_exp = q_exp.split("\n")
    q_exp.pop(-1)
    for i in range(len(q_exp)):
        tmp = q_exp[i].split(", ") 
        q_exp[i] = [float(tmp[0]), float(tmp[1]), float(tmp[2]), float(tmp[3]), float(tmp[4]), float(tmp[5]), float(tmp[6])]



    figsize = (10, 7)
    fontsize = 25
    linewidth = 3
    boxwidth = 1.5
    with plt.style.context(["science", "std-colors"]):
        # plot q7
        fig = plt.figure(figsize=figsize)
        ax1 = fig.add_subplot()
        ax1.set_position([0.1, 0.54, 0.89, 0.38])
        plt.plot(t, q7, label="$q_7$", linewidth=linewidth, color="black")
        plt.plot(t, q7_NN, label="$q_{7,NN}$", linewidth=linewidth, color="grey", linestyle='dashed')

        ax1.autoscale(tight=True)
        ax1.spines["bottom"].set_linewidth(boxwidth)
        ax1.spines["left"].set_linewidth(boxwidth)
        ax1.spines["top"].set_linewidth(boxwidth)
        ax1.spines["right"].set_linewidth(boxwidth)

        ax1.xaxis.set_ticks_position('bottom')
        ax1.yaxis.set_ticks_position('left')
        ax1.tick_params(which='major', width=linewidth, length=6)
        ax1.tick_params(which='minor', width=linewidth, length=3)
        xticks = np.arange(0, round(t[-1]), 2)
        xlabels = []
        ax1.set_xticks(xticks, labels=xlabels, fontsize=fontsize)
        yticks = np.arange(min([min(q7), min(q7_NN)])-ygap, max([max(q7), max(q7_NN)])+ygap*2, ygap)
        ylabels = [f'{y:1.1f}' for y in yticks]
        ax1.set_yticks(yticks, labels=ylabels, fontsize=fontsize)
        plt.ylabel("$q_7$ [rad]", fontsize=fontsize)
        
        #plot O_EE
        ax2 = fig.add_subplot()
        ax2.set_position([0.1, 0.12, 0.89, 0.38])
        plt.plot(t, x, label="$x$", linewidth=linewidth, color='royalblue')
        plt.plot(t, y, label="$y$", linewidth=linewidth, color='limegreen')
        plt.plot(t, z, label="$z$", linewidth=linewidth, color='firebrick')
        plt.plot(t_exp, x_exp, label="$x_{exp}$", linewidth=linewidth, color='deepskyblue', linestyle='dashed')
        plt.plot(t_exp, y_exp, label="$y_{exp}$", linewidth=linewidth, color='lime', linestyle='dashed')
        plt.plot(t_exp, z_exp, label="$z_{exp}$", linewidth=linewidth, color='orangered', linestyle='dashed')

        ax2.autoscale(tight=True)
        ax2.spines["bottom"].set_linewidth(boxwidth)
        ax2.spines["left"].set_linewidth(boxwidth)
        ax2.spines["top"].set_linewidth(boxwidth)
        ax2.spines["right"].set_linewidth(boxwidth)

        ax2.xaxis.set_ticks_position('bottom')
        ax2.yaxis.set_ticks_position('left')
        ax2.tick_params(which='major', width=linewidth, length=6)
        ax2.tick_params(which='minor', width=linewidth, length=3)
        xticks = np.arange(0, round(t_exp[-1]), 2)
        xlabels = [f'{x:1.0f}' for x in xticks]
        ax2.set_xticks(xticks, labels=xlabels, fontsize=fontsize)
        yticks = np.arange(min([min(x), min(y), min(z)])-0.1, max([max(x), max(y), max(z)])+0.2, ygap/2)
        ylabels = [f'{y:1.1f}' for y in yticks]
        ax2.set_yticks(yticks, labels=ylabels, fontsize=fontsize)
        plt.xlabel("t [s]", fontsize=fontsize)
        plt.ylabel("$O_{EE}$ [m]", fontsize=fontsize)

        fig.legend(loc='outside upper center', ncols=8, fontsize=fontsize, handlelength=1, borderpad=0.0, handletextpad=0.4, borderaxespad=0.2, columnspacing=1)
        fig.text(0.01, 0.88, "(a)", fontsize=fontsize)
        fig.text(0.01, 0.46, "(b)", fontsize=fontsize)

        if show:
            plt.show()
        fig.savefig(f'{pack_path}/data/plots/{traj}.pdf', dpi=300)
        plt.close()


       

if __name__ == "__main__":
    if len(sys.argv) > 1:
        for traj in sys.argv[1:]:
            plotter(traj)
        



            

    







    
    


            


