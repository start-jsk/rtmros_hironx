#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro019)] [loglevel: NORMAL, DEBUG, TRACE, VERBOSE]"
    echo >&2 "          [-h|--help] Print help message."
    exit 0
}

# command line parse. If the num of argument is not as expected, call usage func.
OPT=`getopt -o h -l help -- $*`
if [ $# != 2 ]; then
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
loglevel=${2:NORMAL}
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
  grep -ci \"^logger.log_level:\s*${loglevel}\" ${CONF_FILE} && (echo '* Current log level is already ${loglevel}'; exit 1);

  echo '* Create backup of ${CONF_FILE}';
  cp ${CONF_FILE} ${BACK_FILE};

  set -x;
  echo '* Add patch to ${CONF_FILE}';
  sed -i 's@^logger.log_level:@#logger.log_level:@' ${CONF_FILE};
  sed -i 's@^#logger.log_level:\s*${loglevel}@logger.log_level: ${loglevel}@' ${CONF_FILE};

  echo '* Check updated diff';
  diff ${BACK_FILE} ${CONF_FILE} || echo \"* Succeeded to update\";
  echo '* Check updated file';
  ls -al ${CONF_FILE};  
  cat ${CONF_FILE};
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
