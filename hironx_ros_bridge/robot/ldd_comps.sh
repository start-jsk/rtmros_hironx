#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro019)]"
    echo >&2 "          [-h|--help] Print help message."
    exit 0
}

# command line parse. If the num of argument is not as expected, call usage func.
OPT=`getopt -o h -l help -- $*`
if [ $# != 1 ]; then
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

hostname=$1
hostname=${hostname:="hiro019"} 
ossuser_qnx="tork"
DATE=`date +"%Y%m%d-%H%M%S"`

# Command that gets run on QNX. 
# - (Assumes remotely logged in to qnx via ssh)
# - Create tarball of /opt/jsk/var/log/* at /home/tork
# - Log out of QNX
# - scp the tarball
commands="
  . ~/.profile || echo '~/.profile does not exist. Move on anyway.';
  env;
  trap 'exit 1' ERR;
  set -x;

  df;

  find /opt/jsk -not -type d -exec ls -al {}\; ;

  find /opt/jsk/bin/ -iname \"*Comp\" -exec ls -al {} \; -exec ldd {} \; ;

  find /opt/jsk/lib/ -iname \"*.so\" -exec ls -al {} \; -exec ldd {} \; ;

  exit;
  "

IS_SUCCESS=1

NAME_LOGFILE=$0_${DATE}.log
read -p "Run the command @ $hostname (y/n)? "
if [ "$REPLY" == "y" ]; then
    ssh $ossuser_qnx@$hostname -t $commands 2>&1 | tee ${NAME_LOGFILE}

    IS_SUCCESS=0
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

if [ "${IS_SUCCESS}" -eq 1 ]; then
    echo "Operation unsuccessful. ..... "
fi
echo "Done : Send back log file: ${NAME_LOGFILE}"
