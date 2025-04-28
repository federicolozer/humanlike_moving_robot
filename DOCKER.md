# Instructions to launch the Human-Like Moving Robot demo

First, enable access to X server for everyone with the following command:
```shell script
sudo xhost +
```

Then run the latest `humanlike-moving-robot` image as a Docker container.
Run the followin command:
```shell script
sudo docker run -it -e DISPLAY=$DISPLAY --net host -v /tmp/.X11-unix:/tmp/.X11-unix --privileged federicolozer/humanlike-moving-robot:latest
```

From this tab you can run the command:
```shell script
roslaunch humanlike_moving_robot main.launch
```

Since you have to launch two different commands to run the demo, you need two different terminals.
So you just need to connect to the running container from another tab.

Just oper a new tab on the host machine (not from the Docker container) and run the following command:
```shell script
sudo docker ps
```
This command will list all the running containers.
Copy the number at the voice "CONTAINER ID".

The connect to the container by running the following command:
```shell script
sudo docker exec -it [CONTAINER ID] bash
```
**Note**: Remember to replace `[CONTAINER ID]` with the number you copied before.

From this tab you can run the command:
```shell script
rosrun humanlike_moving_robot GUI.py
```
