#! /bin/bash

# Basic stuff
sudo apt-get update
sudo apt-get install -y build-essential vim emacs git tmux python-dev curl python-pip cmake libgif-dev openssh-server

# ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get -y install ros-indigo-desktop-full
#echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
sudo rosdep init
rosdep update

# MoveIt
sudo apt-get install -y ros-indigo-moveit-*

# Fetch
sudo apt-get install -y ros-indigo-fetch-*

# ROS utils
sudo apt-get install -y python-wstool python-catkin-tools python-rosinstall

# NVM
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.8/install.sh | bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" # This loads nvm
nvm install node
npm install -g polymer-cli bower

# Caddy
curl https://getcaddy.com | bash -s personal http.cors
