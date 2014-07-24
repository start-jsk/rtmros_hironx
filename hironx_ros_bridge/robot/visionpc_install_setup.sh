#!/bin/sh

#function show_usage {
#    echo >&2 "usage: $0 [user accout (default:tork)]"
#    echo >&2 " [-h|--help] print this message"
#    exit 0
#}

# command line parse
#OPT=`getopt -o h -l help -- $*`
#if [ $? != 0 ]; then
#    # If no arg, run show_usage function.
#    show_usage
#fi

DIST_PRECISE="Precise"
DIST_TRUSTY="Trusty"
DISTRO=$DIST_PRECISE

# ROS pre-download setup. See http://wiki.ros.org/hydro/Installation/Ubuntu for detail.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update

# Define packages to be installed.
PKG_TO_INSTALL=""

# For oss development
PKG_OSS_DEV="git gitk meld"
PKG_TO_INSTALL="$PKG_TO_INSTALL $PKG_OSS_DEV"

# For ROS
PKG_ROS="ros-hydro-desktop-full ros-hydro-rtmros-nextage ros-hydro-hironx-tutorial ros-hydro-rtshell-core ntp"
PKG_TO_INSTALL="$PKG_TO_INSTALL $PKG_ROS"

# Random tools
PKG_RANDOM_TOOLS="ack-grep sysinfo synaptic"
PKG_TO_INSTALL="$PKG_TO_INSTALL $PKG_RANDOM_TOOLS"

echo Installing $PKG_TO_INSTALL
sudo apt-get install -y $PKG_TO_INSTALL

# ROS post-download setting
echo "## For ROS."
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init
rosdep update

# Connect Ubuntu to the robot. IP Addr and user name needs modified accordingly.
sudo sh -c 'echo "# For connection to the robot. See https://github.com/start-jsk/rtmros_hironx/issues/136#issuecomment-48106612" >> /etc/hosts'
sudo sh -c 'echo "192.168.128.10  nextage" >> /etc/hosts'

# Remove unnecessary folders on home dir.
cd ~
rm -fr Documents Downloads examples.desktop Music Pictures Public Templates Videos

# Create a developer user on Ubuntu with admin privilege.
sudo useradd -d /home/nxouser -s /bin/bash -m nxouser && sudo adduser nxouser sudo
sudo passwd nxouser
# Setup ROS environment for nxouser
echo "source /opt/ros/hydro/setup.bash" >> /home/nxouser/.bashrc
