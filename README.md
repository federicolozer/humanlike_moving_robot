<meta name="viewport" content="width=device-width, initial-scale=1">
<div align="center">
  <img src=data/media/logo_ADMiRE.png alt="logo ADMiRE" height=150pt style="display: block; margin: 0 auto">
  <img src=data/media/logo_Uniud.png alt="logo Uniud" height=150pt style="display: block; margin: 0 auto">
</div>

# Human-like Moving Robot

<div align="center">
  <img src=data/media/Panda.png alt="Panda robot" width=50% style="display: block; margin: 0 auto">
</div>

<p>
  This is a package developed by ADMiRE Research Center in collaboration with University of Udine.
  The aim is to reproduce the human behavior on a redundant collaborative robot. 
  This application is designed specifically for Franka Emika Panda robot.
</p>

## Prerequisites

| **OS** | Ubuntu 18.04 |
| - | - |
| **ROS Distro** | Melodic Morenia |
| **Python** | version 3.8 |
| **Pip** | version 25.0.1 |
| **Numpy** | version 1.24.4 |
| **Torch** | version 2.4.1+cpu |
| **Flask** | version 3.0.3 |
| **Scipy** | version 1.10.1 |
| **Rospkg** | version 1.6.0 |
| **Pyyaml** | version 6.0.2 |

## Instructions for the setup

### Updates
Before starting, it is a good practice to check for updates and to upgrade all the packages in the PC.
To do that, run the following commands:
```shell script
sudo apt update
sudo apt upgrade -y
```
**Note:** You will be asked to enter your password to execute ccommands as super user.

### Install ROS environment
To work with this package, you need to install ROS Melodic.
You can find the whole procedure [**here**](https://wiki.ros.org/melodic/Installation/Ubuntu).

After you have installed ROS, at the end of the page, you were asked to update `rosdep`.
Since Melodic is and end-of-life distro, will be skipped.
To include it, run the following command:
```shell script
rosdep update --rosdistro melodic
```   

### Install Python dependencies
To run the latest `PyTorch` on *Ubuntu*, you need to install `Python 3`, version >= 3.9.
Since the latest supported version for *Ubuntu 18.04 is `Python 3.8`, you need to install a previous version of `PyTorch` compatible with that specific `Python 3` version, 2.4.1 for instance.

It is recommended to work with a `Python 3.8` virtual environment if you have a different version of `Python 3` installed on your PC.
To install, set up and launch the virtual environment, run the following commands:
```shell script
sudo apt install python3.8-venv -y
python3.8 -m venv ~/your-venv-name
source ~/your-venv-name/bin/activate
```
**Note:** Remember to replace "your-venv-name" with a name of your choise for your virtual environment.

Instead, if you want to install `Python 3.8` directly on your machine, run the following commands:
```shell script
sudo apt install python3.8
```
**Warning:** This operation is not reccomended because `Python 3` in the scripts shebangs will still refer to your old version, even if you use an alias.
You can alternatively replace all the shebangs or try to fix the issue in another way.

Then, install `pip3`:
```shell script
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python3.8 get-pip.py
```
Check your `pip3` version with the following command:
```shell script
pip3 --version
```
If you are not able to see it, add the installation directory to PATH environment variable with the following command:
```shell script
PATH="~/.local/bin:$PATH"
```
**Note:** Remember to replace "your-user-name" with your real username.

Let´s install some other required packages with the following commands:
```shell script
sudo apt install python3.8-dev -y
pip3 install numpy scipy flask rospkg pyyaml progress
```

To install  `PyTorch 2.4.1`, run the following command:
```shell script
pip3 install torch==2.4.1 --index-url https://download.pytorch.org/whl/cpu
```

### Install and build the package
The first thing to do is to create a ROS workspace.
If you already have one, you can skip this part, otherwise run the following commands.

Firts, source the setup file:
```shell script
source /opt/ros/melodic/setup.bash
```
If you have already inserted it into your *~/.bashrc* file, you don't need to do it.

Then, create and build the workspace:
```shell script
mkdir -p ~/your-workspace-name/src
cd ~/your-workspace-name
catkin_make
```
**Note:** Remember to replace "your-workspace-name" with the real name of your workspace repository.

Finally, source your new setup file:
```shell script
source devel/setup.bash
```
For convenience, you can add the command to your *~/.bashrc* file, to make sure it will be executed every time you open the terminal.
Ýou can do it by running the following command:
```shell script
echo "source ~/your-workspace-name/devel/setup.bash" >> ~/.bashrc
```
**Note:** Remember to replace "your-workspace-name" with the real name of your workspace repository.

To install the package from this GitHub page, go into the *src* folder and run the git clone command:
```shell script
cd src
git clone https://github.com/federicolozer/humanlike_moving_robot.git
```

Before building the package, ensure you have the `Eigen` library on your pc.
Otherwise you can download it and install it from [**here**](https://eigen.tuxfamily.org/index.php?title=Main_Page#Download)

To build the package, come back to the workspace folder and run the make command:
```shell script
cd ..
catkin_make -DPYTHON_LIBRARY:FILEPATH=/usr/lib/x86_64-linux-gnu/libpython3.8.so -DPYTHON_INCLUDE_DIR:PATH=/usr/include/python3.8
```

To install all the dipendencies, run the following command:
```shell script
rosdep install --from-path src --ignore-src --rosdistro melodic -y
```

Add your `Python` script folder to `PYTHONPATH` environment variable, to make some files recognize the path of some module to load.
Run the following command:
```shell script
PYTHONPATH=~/your-workspace-name/src/humanlike_moving_robot/scripts:$PYTHONPATH
```
You can add also this command to your *~/.bashrc* file, for the next time, with the following command:
```shell script
echo PYTHONPATH=\"~/your-workspace-name/src/humanlike_moving_robot/scripts:\$PYTHONPATH\" >> ~/.bashrc
```
**Note:** Remember to replace "your-workspace-name" with the real name of your workspace repository.

## Get started
To start the demo, make sure you are into the virtual environment.
Run the following command:
```shell script
source ~/your-venv-name/bin/activate
```
**Note:** Remember to replace "your-venv-name" with a name of your choise for your virtual environment.

Since you have to run two commands, prepare two different tabs, each one with the virtual environment open inside.

In the first tab, to launch the simulation environment and the required servers, run the following command:
```shell script
roslaunch humanlike_moving_robot main.launch
```
Instead, if you want to use the physic robot, run the following command:
```shell script
roslaunch humanlike_moving_robot main.launch use_real_robot:=true
```

In the second tab, to open the GUI with the main program, run the following command:
```shell script
rosrun humanlike_moving_robot GUI.py
```

Otherwise, you can just run the main program and select the trajectories to perform from command line.
In this case, run the following command:
```shell script
rosrun humanlike_moving_robot main.py
```










