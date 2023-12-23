#!/bin/bash

container_name=f1tenth_stack
function create_container {
        docker run --rm -it\
        --name ${container_name} \
        -h ${container_name} \
        --env="DISPLAY"\
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --privileged \
        --network=host \
        f1tenth:gym_ros_foxy
}

function rm_container {
	if [ "$(docker ps -aq -f name=${container_name})" ]
        then
		if [ "$(docker ps -aq -f status=running)" ]
		then
			docker stop ${container_name}
		fi
        	docker rm ${container_name}
        fi
}

if [ !"$(docker ps -aq -f name=${container_name})" ]
then 
	xhost +local:root
	create_container
	xhost -local:root
fi

if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
then
	echo "Container is Running. Starting new session."
	docker exec -it ${container_name} bash 
fi
