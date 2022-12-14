#docker run --name wheeler3 --hostname wheeler3 --mount type=bind,source=/home/n800s,target=/home/n800s  -p 11611:11611 -p 11311:11311 -p38031:38031 -it ros1wheeler3_1

docker run --name wheeler3 --network host --mount type=bind,source=/home/n800s,target=/home/n800s -v /dev:/dev --privileged \
	--add-host rpi2:127.0.0.1 --add-host gate:192.168.1.50 --add-host big:192.168.1.5 --restart unless-stopped \
	-it ros1wheeler3_1

