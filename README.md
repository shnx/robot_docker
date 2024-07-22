# Pioneer 2DX
#### Reason for this repo
Working with docker was not easy until I had to communicate with old mobile robot for my theis work. 
Here I will explain how to setup a ros noetic container
#### Install docker 

##### sudo apt install docker.io

##### snap version
###### better to update it
##### sudo apt install snapd
##### sudo snap install docker
##### sudo systemctl status docker
##### sudo docker run -it --net=host --device=/dev/dri:/dev/dri   --env="DISPLAY"   --env="QT_X11_NO_MITSHM=1"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --name=robot   osrf/ros:noetic-desktop-full bash -it
#### Prepare PC to accept X11
##### xhost +
#### New terminal 
#### docker start robot
#### docker exec -it robot bash
##### or sudo docker run -it --net=host   --env="DISPLAY"   --env="QT_X11_NO_MITSHM=1"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --volume="$HOME/Documents:$HOME/Documents:rw"   --device=/dev/ttyUSB0:/dev/ttyUSB0   --device=/dev/ttyUSB1:/dev/ttyUSB1   --device=/dev/ttyUSB2:/dev/ttyUSB2 --device=/dev/input/js0  --name robot_6   osrf/ros:melodic-desktop-full   /bin/bash


### running the code 
install gmapping , move-base
