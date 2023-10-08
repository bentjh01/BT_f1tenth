#!/bin/bash

if [ ! "$(docker ps -aq -f name=f1tenth_gym_ros)" ]; then
	echo Creating Container
	docker run -dt \
		--name f1tenth_gym_ros \
		-h f1tenth_gym_ros \
		-v $(pwd)/mnt/f1tenth_gym_ros:/root/mnt_f1tenth_gym_ros \
		--env="DISPLAY"\
    		--env="QT_X11_NO_MITSHM=1" \
    		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--net=host \
		f1tenth:gym_ros_foxy

elif [ "$(docker ps -aq -f status=exited -f name=f1tenth_gym_ros)" ] ; then
        echo Start Container
        docker start f1tenth_gym_ros
fi

if [ "$(docker ps -aq -f status=running -f name=f1tenth_gym_ros)" ] ; then
        echo Container Running. Starting new terminal. 
        docker exec -it f1tenth_gym_ros bash
fi

#-e DISPLAY=host.docker.internal:0.0 \
