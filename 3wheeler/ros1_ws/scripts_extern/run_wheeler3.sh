#docker run --name wheeler3 --hostname wheeler3ext \
#	--mount type=bind,source=/home/n800s,target=/home/n800s \
#	-p 11311:11311 -p 11611:11611 -p38031:38031 -it ros1wheeler3_1

docker run --name wheeler3 --network host --mount type=bind,source=/home/n800s,target=/home/n800s \
	--add-host rpi2:192.168.1.59 --add-host big:192.168.1.5 \
	-it ros1wheeler3_1
