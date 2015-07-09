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
CONF_FILE='/opt/jsk/etc/HIRONX/hrprtc/rtcdRobotMode.conf'
BACK_FILE=${CONF_FILE}-${DATE}.bak;
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
  set +x;

  ls -al ${CONF_FILE};
  cat ${CONF_FILE};

  echo '* Check if patch is already applied ${CONF_FILE}';
  grep -ci \"example.AbsoluteForceSensor.config_file\" ${CONF_FILE} && (echo '* Patch is already applied'; exit 1);

  echo '* Create backup of ${CONF_FILE}';
  cp ${CONF_FILE} ${BACK_FILE};

  echo '* Add patch to ${CONF_FILE}';
  echo \"example.AbsoluteForceSensor.config_file: /opt/jsk/etc/HIRONX/hrprtc/Robot.conf\" >> ${CONF_FILE};

  echo '* Check updates ';
  diff ${BACK_FILE} ${CONF_FILE};
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
