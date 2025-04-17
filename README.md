# Instructions to get ready with minimum time-jerk trajectory tests



| **Prerequisites** | |
| - | - |
| **OS** | Ubuntu 18.04 |
| **ROS Distro** | Melodic Morenia |



## Updates
Before starting, it is a good practice to check for updates and to upgrade all the packages in the PC.
To do that, launch:
```shell script
sudo apt update
sudo apt upgrade -y
```
**Note:** You will be asked to enter your password to execute ccommands as super user.



## Install ROS environment
To work with this package, you need to install ROS Melodic.
You can find the whole procedure [**here**](https://wiki.ros.org/melodic/Installation/Ubuntu).

After you have installed ROS, at the end of the page, you were asked to update `rosdep`.
Since Melodic is and end-of-life distro, will be skipped.
To include it, launch the following command:
```shell script
rosdep update --include-eol-distros
```



## Install and build the package
The first thing to do is to create a ROS workspace.
If you already have one, you can skip this part, otherwise launch the following commands.

Firts, source the setup file:
```shell script
source /opt/ros/melodic/setup.bash
```
If you have already inserted it into your *~/.bashrc* file, you don't need to do it.

Then, create and build the workspace:
```shell script
mkdir -p ~/your-package-name/src
cd ~/your-package-name
catkin_make
```
**Note:** Remember to replace "your-workspace-name" with the real name of your workspace repository.

Finally, source your new setup file:
```shell script
source devel/setup.bash
```
For convenience, you can add the command to your *~/.bashrc* file, to make sure it will be executed every time you open the terminal.
Ýou can do it launching the following command:
```shell script
echo "source ~/your-package-name/devel/setup.bash" >> ~/.bashrc
```

To install the package from this GitHub page, go into the *src* folder and launch the git clone command:
```shell script
cd src
git clone https://github.com/federicolozer/humanlike_moving_robot.git
```

To build the package, come back to the workspace folder and launch the make command:
```shell script
cd ..
catkin_make -DPYTHON_LIBRARY:FILEPATH=/usr/lib/x86_64-linux-gnu/libpython3.8.so -DPYTHON_INCLUDE_DIR:PATH=/usr/include/python3.8
```

To automate some processes, launch the following commands:
```shell script
echo PYTHONPATH=\"\$HOME/your-workspace-name/src/humanlike_moving_robot/scripts:\$PYTHONPATH\" >> ~/.bashrc
echo source /opt/ros/melodic/setup.bash >> ~/.bashrc
echo source /home/lozer/your-workspace-name/devel/setup.bash >> ~/.bashrc
```
Remember to replace "your-workspace-name" with the real name of your workspace repository.





## Install Python3.8
To run `PyTorch`, you need to install  


