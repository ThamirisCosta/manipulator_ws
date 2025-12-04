#!/bin/bash

IMAGE_NAME="jazzy-image"
CONTAINER_NAME="ros-jazzy-container"
LOCAL_WORKSPACE="$HOME/ros2_workspace"

XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Verifica se NVIDIA está disponível
if docker run --rm --gpus all nvidia/cuda:12.3.0-base nvidia-smi &>/dev/null; then
    echo "🔋 NVIDIA detectada — habilitando GPUs no container!"
    GPU_FLAGS="--gpus all \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all"
else
    echo "⚠️ NVIDIA NÃO disponível — rodando sem aceleração 3D!"
    GPU_FLAGS=""
fi

docker run \
    --name $CONTAINER_NAME \
    --rm \
    -it \
    --network host \
    --privileged \
    $GPU_FLAGS \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTH:$XAUTH \
    -v /etc/localtime:/etc/localtime:ro \
    -v $LOCAL_WORKSPACE:/ros2_ws \
    --device /dev/dri \
    --group-add video \
    --security-opt seccomp=unconfined \
    $IMAGE_NAME
