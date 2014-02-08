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

commands="
  echo \"* Reboot Computer *\";
  shutdown
  "

hostname=$1
hostname=${hostname:="hiro014"} 
echo "comands = $commands"
read -p "execute controllers restart command @ $hostname (y/n)?"
if [ "$REPLY" == "y" ]; then
    ssh root@$hostname -t $commands
else
    echo "DO NOT RUN"
    echo "EXITTING.."
fi

