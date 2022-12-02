sudo systemctl stop fastdds
sudo systemctl disable fastdds.service
sudo rm /etc/systemd/system/fastdds.service
sudo systemctl daemon-reload
