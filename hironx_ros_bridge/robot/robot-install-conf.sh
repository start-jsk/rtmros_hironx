#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro014)]"
    echo >&2 "          [-h|--help] print this message"
    exit 0
}

# command line parse
OPT=`getopt -o h -l help -- $*`
if [ $? != 0 ]; then
    usage
fi

eval set -- $OPT

while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

domain=`host rtm-ros-robotics.googlecode.com | awk '/^[[:alnum:].-]+ has address/ { print $4 ; exit }'`

commands="
  echo \"* Download hironx_ros_bridge/robot *\";
  mkdir -p src
  cd src
  svn co https://$domain/svn/trunk/rtmros_hironx/hironx_ros_bridge/robot robot
  echo \"* Configure configure files *\";
  cd robot;
  PATH=/opt/jsk/bin:$PATH make
  echo \"* Install configure files *\";
  PATH=/opt/jsk/bin:$PATH su -c 'make install'
  "
hostname=$1
hostname=${hostname:="hiro014"} 
read -p "execute install command @ $hostname (y/n)?"
if [ "$REPLY" == "y" ]; then
    ssh root@$hostname -t $commands
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

