# Instructions for setup with Docker

To launch the application from a Docker container, you just need to download a preconfigured image.

To pull the latest `humanlike-moving-robot` image from DockerHub, run the followin command:
```shell script
sudo docker pull federicolozer/humanlike-moving-robot:latest
```

You also have to enable access to X server for everyone with the following command:
```shell script
sudo xhost +
```
**Note**: You don't need it if you are running the docker container from wsl.

Then run the latest `humanlike-moving-robot` image as a Docker container.
Run the followin command:
```shell script
sudo docker run -it -e DISPLAY=$DISPLAY --net host -v /tmp/.X11-unix:/tmp/.X11-unix --privileged federicolozer/humanlike-moving-robot:latest
```

From this tab you can run the following command, for instance:
```shell script
roslaunch humanlike_moving_robot main.launch
```

Since you have to launch two different commands to run the demo, you need two different terminals.
So you just need to connect to the running container from another tab.

Just open a new tab on the host machine (not from the Docker container) and run the following command:
```shell script
sudo docker ps
```
This command will list all the running containers.

Copy the number at the voice "CONTAINER ID".

Then connect to the container by running the following command:
```shell script
sudo docker exec -it [CONTAINER ID] bash
```
**Note**: Remember to replace `[CONTAINER ID]` with the number you copied before.

From this tab you can run another command, like the following one, for instance:
```shell script
rosrun humanlike_moving_robot GUI.py
```
