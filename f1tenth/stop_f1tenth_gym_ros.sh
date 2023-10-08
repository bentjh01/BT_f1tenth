#!/bin/bash

if [ "$(docker ps -aq -f status=running -f name=f1tenth_gym_ros)" ] ; then
        echo Container Running. Stopping Container. 
        docker stop f1tenth_gym_ros
fi

if [ "$(docker ps -aq -f status=exited -f name=f1tenth_gym_ros)" ] ; then
        echo Container Stopped. Removing Container.
        docker rm f1tenth_gym_ros
fi
