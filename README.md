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
##### sudo system status docker
##### sudo docker run -it --net=host --device=/dev/dri:/dev/dri   --env="DISPLAY"   --env="QT_X11_NO_MITSHM=1"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --name=robot   osrf/ros:noetic-desktop-full bash -it
#### Prepare PC to accept X11
##### xhost +
#### New terminal 
#### docker exec -it robot bash
