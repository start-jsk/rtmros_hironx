#!/bin/bash

## Discussion for this script: https://github.com/start-jsk/hironx-package/issues/33
## Install base components of Hiro 2-armed robot. 
## @arg hostname_robot
## @arg ?

# Create a work folder for Hiro
path_hironx_work=~/hironx_rsc
# Download directory for new tarball
path_dl=$path_hironx_work/tmp_hironx_dl
# Tarball that goes into binary folder (eg. /opt/jsk)
url_tarball=http://rtm-ros-robotics.googlecode.com/files/
filename_tarball=hironx_opt_jsk_latest.tar.gz

# For stashing
dir_stash=old_rsc
dir_install_target=/opt/jsk
timestamp=$(date +\"%Y%m%d%H%M%S\");

# If wget doesn't exist, program terminates and urge user to install.
command_dl_tarball="
  echo \"This script uses (and creates if it's not yet done so) directory $path_hironx_work\";
  mkdir $path_hironx_work;
  mkdir $path_dl;
  cd $path_dl;
  trap 'ERROR wget failed to retrieve $url_tarball$filename_tarball; rm $filename_tarball; exit' ERR;
  /usr/pkg/bin/wget $url_tarball$filename_tarball -O $filename_tarball;
  trap - ERR;
  tar fvxz $filename_tarball;
  rm $filename_tarball;

  echo \"Stashing directory $dir_install_target to $dir_stash\";
  cd $path_hironx_work;
  mkdir $dir_stash;
  cd $dir_stash;
  mkdir hironx_$timestamp;
  cd hironx_$timestamp;
  su -c 'mv $dir_install_target/* .';
  echo \"Previous files are stashed away at: $path_hironx_work/$dir_stash/hironx_$timestamp \";

  echo \"Moving new files into $dir_install_target \";
  trap 'ERROR could not move new resource into $dir_install_target...; exit' ERR;
  su -c 'mv $path_dl/* $dir_install_target';
  trap - ERR;
  echo \"New moved files are:\";
  ls -lt $dir_install_target;
  "
## For some reason wget needs full path like this, otherwise error:
##  sh: wget: cannot execute - No such file or directory

cmd1=$command_dl_tarball
commands_all=$cmd1

instruction_arg=" ARG#1: hostname/ipaddr (DEFAULT: hiro014), ARG#2: username (DEfAULT: hiro)"

echo "NOTE: This script will try to expand into $dir_install_target. Edit the dir_install_target variable in the script file if it's not what you want."
echo $instruction_arg
hostname=$1
hostname=${hostname:="hiro014"} 
userid=$2
userid=${userid:="hiro"} 
#hostname=${hostname:="192.168.1.13"}  #Local QNX on VM for testing
read -p "Move aside the current Hiro resouce @ robot's hostname: $hostname (y/n)?"
if [ "$REPLY" == "y" ]; then
    echo "command to be run: $commands_all"
    ssh $userid@$hostname -t $commands_all
else
    echo "Aborted."
    echo "Consider typing appropriate arguments upon running this script."
    echo "$instruction_arg"
fi
