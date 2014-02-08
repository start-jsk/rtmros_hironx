#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro014)]"
    echo >&2 "       $1 [user name (default:hiro)]"
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

domain=`host rtm-ros-robotics.googlecode.com | awk '/^[[:alnum:].-]+ has address/ { print $4 ; exit }'` # this does not work for  Server certificate verification 

commands="
  . ~/.profile;
  echo \"* Download hironx_ros_bridge/robot *\";
  mkdir -p src;
  cd src;
  svn co http://rtm-ros-robotics.googlecode.com/svn/trunk/rtmros_hironx/hironx_ros_bridge/robot robot;
  echo \"* Configure configure files *\";
  cd robot;
  make;
  echo \"* Install configure files *\";
  su -c 'make install';
  "
hostname=$1
hostname=${hostname:="hiro014"} 
userid=$2
userid=${userid:="hiro"} 
echo "comands = $commands"
read -p "execute install command @ $hostname (y/n)?"
if [ "$REPLY" == "y" ]; then
    ssh $userid@$hostname -t $commands
else
    echo "DO NOT RUN commands"
    echo "EXITTING.."
fi

