#!/usr/bin/env bash

echo "Running build_team_system.bash for figment_team"

# Prepare ROS
. /opt/ros/kinetic/setup.bash

# Install the necessary dependencies for getting the team's source code
# Note: there is no need to use `sudo`.
apt-get update
apt-get install -y wget unzip

# Create a catkin workspace
mkdir -p ~/figment/src
rm -rf ~/figment/src/*

# Fetch the source code for our team's code
cd ~/figment/src
wget https://bitbucket.org/figment-gprt/figment/get/master.zip
unzip master.zip

cd ~/figment
catkin_make install


