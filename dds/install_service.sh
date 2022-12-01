chmod 664 fastdds.service
sudo rm /etc/systemd/system/fastdds.service
sudo cp "`pwd`/fastdds.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable fastdds.service
sudo systemctl start fastdds
