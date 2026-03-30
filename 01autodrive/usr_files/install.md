# install jetson-stats, for monitor hardware, e.g. CPU, GPU, RAM, Disk
sudo -H pip3 install -U jetson-stats
sudo systemctl restart jetson_stats.service

sudo apt-get install python3-pcl
sudo apt-get install ros-foxy-sensor-msgs-py
sudo apt-get install -y  libpcap-dev
sudo apt-get install -y libyaml-cpp-dev
sudo apt-get install -y libprotobuf-dev protobuf-compiler
reboot
