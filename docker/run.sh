#!/bin/sh

# recover the name of the image
IMAGE_NAME=$(cat build.sh | grep IMAGE_NAME | grep -v build)
IMAGE_NAME=${IMAGE_NAME#*=}

# problems whith visualization
xhost +
docker run --gpus all -ti --rm --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged -v $1:/working_dir ${IMAGE_NAME} 
