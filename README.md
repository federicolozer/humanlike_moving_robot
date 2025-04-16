# Instructions to get ready with minimum time-jerk trajectory tests


## Install and build the package
The first thing to do is to create a ROS workspace.
If you already have one, you can skip this part, otherwise launch the following commands.
Firts, source the setup file:
```shell script
source /opt/ros/melodic/setup.bash
```
Then, create and build the workspace:
```shell script
mkdir -p ~/your-package-name/src
cd ~/your-package-name
catkin_make
```
Remember to replace "your-workspace-name" with the real name of your workspace repository.
Finally, source your new setup file:
```shell script
source devel/setup.bash
```

To install the package from this GitHub page, go into the src folder and launch the git clone command:
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




