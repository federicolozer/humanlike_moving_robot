# Instructions to get ready with minimum time-jerk trajectory tests



| **Prerequisites** | |
| - | - |
| **OS** | Ubuntu 18.04 |
| **ROS Distro** | Melodic Morenia |
| **Python** | version 3.8 |
| **Pip** | version 25.0.1 |
| **Numpy** | version 1.24.1 |
| **Torch** | version 2.4.1+cpu |
| **Flask** | version 3.0.3 |
| **Scipy** | version 1.10.1 |



## Updates
Before starting, it is a good practice to check for updates and to upgrade all the packages in the PC.
To do that, run the following commands:
```shell script
sudo apt update
sudo apt upgrade -y
```
**Note:** You will be asked to enter your password to execute ccommands as super user.



## Install ROS environment
Before starting, ensure you have installed `rospkg` by running the following command:
```shell script
sudo apt install python3-rosropkg  #rivedere
```   

To work with this package, you need to install ROS Melodic.
You can find the whole procedure [**here**](https://wiki.ros.org/melodic/Installation/Ubuntu).

After you have installed ROS, at the end of the page, you were asked to update `rosdep`.
Since Melodic is and end-of-life distro, will be skipped.
To include it, run the following command:
```shell script
rosdep update --rosdistro melodic
```   



## Install PyTorch
To run the latest `PyTorch` on *Ubuntu*, you need to install `Python 3`, version >= 3.9.
Since the latest supported version for *Ubuntu 18.04 is `Python 3.8`, you need to install a previous version of `PyTorch` compatible with that specific `Python 3` version, 2.4.1 for instance.

It is reccomended to work with a `Python 3.8` virtual environment if you have a different version of `Python 3` installed on your PC.
To install, set up and launch the virtual environment, run the following commands:
```shell script
sudo apt install python3.8-venv
python3.8 -m venv your-venv-name
source your-venv-name/bin/activate
```
**Note:** Remember to replace "your-venv-name" with the real name of your virtual environment.

Instead, if you want to install `Python 3.8` directly on your machine, run the following commands:
```shell script
sudo apt install python3.8
```
#**Note:** Scripts shebangs will still refer to your old `Python 3` version. You got to change them all or to make `Python 3` refer to `Python 3.8`
#For the second option, run the following commands:
#```shell script
#alias python3=/usr/bin/python3.8
#```

Then, install `pip`:
```shell script
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python3.8 get-pip.py
```

```shell script
alias python3=/usr/bin/python3.8
alias pip3=pip3.8
```

Let´s install some other required packages with the following commands:
```shell script
sudo apt install python3.8-dev
pip3 install numpy==1.24.1
pip3 install --upgrade scipy
pip3 install --upgrade flask
```

To install  `PyTorch 2.4.1`, run the following command:
```shell script
pip3 install torch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 --index-url https://download.pytorch.org/whl/cpu
```




## Install and build the package
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
Ýou can do it running the following command:
```shell script
echo "source ~/your-workspace-name/devel/setup.bash" >> ~/.bashrc
```
**Note:** Remember to replace "your-workspace-name" with the real name of your workspace repository.

To install the package from this GitHub page, go into the *src* folder and run the git clone command:
```shell script
cd src
git clone https://github.com/federicolozer/humanlike_moving_robot.git
```

To build the package, come back to the workspace folder and run the make command:
```shell script
cd ..
catkin_make -DPYTHON_LIBRARY:FILEPATH=/usr/lib/x86_64-linux-gnu/libpython3.8.so -DPYTHON_INCLUDE_DIR:PATH=/usr/include/python3.8
```

To install all the dipendencies, run the following command:
```shell script
rosdep install --from-path src --ignore-src --rosdistro melodic
```

#To automate some processes, run the following commands:
#```shell script
#echo PYTHONPATH=\"\$HOME/your-workspace-name/src/humanlike_moving_robot/scripts:\$PYTHONPATH\" >> ~/.bashrc
#echo source /opt/ros/melodic/setup.bash >> ~/.bashrc
#echo source /home/lozer/your-workspace-name/devel/setup.bash >> ~/.bashrc
#```
#**Note:** Remember to replace "your-workspace-name" with the real name of your workspace repository.









