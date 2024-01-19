#!/bin/sh

container_name=f1tenth_bt
function create_container {
        docker run -it\
        --name ${container_name} \
        -h ${container_name} \
        --network=host \
		--volume="$(pwd)/mnt":"/mnt" \
        bentjh01:f1tenth-foxy
}
function rm_container {
	if [ "$(docker ps -aq -f name=${container_name})" ]
        then
		if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
		then
			docker stop ${container_name}
		fi
        	docker rm ${container_name}
        fi
}

if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
then
	echo "Container is Running. Starting new session."
	docker exec -it ${container_name} bash 
else
	rm_container
	create_container
fi
