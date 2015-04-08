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
OSS_FOLDER='/opt/jsk'
OSS_FOLDER_LOG=${OSS_FOLDER}/var/log
DATE=`date +"%Y%m%d-%H%M%S"`
TMP_FOLDER_ROOT=/tmp/HiroNXO_Log
NAME_LOGFILE=${TMP_FOLDER_ROOT}/qnx_fetch_log_${hostname}_${DATE}.log

mkdir ${TMP_FOLDER_ROOT}

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

  echo '* Create tarball of ${OSS_FOLDER_LOG} *';
  tar cfvz opt_jsk_var_logs_${DATE}.tgz ${OSS_FOLDER_LOG}/* || echo '* Failed to create tarball *';

  echo '* Exitting ssh session to QNX. *';
  exit;
  "

IS_SUCCESS=1

read -p "Run the command @ $hostname (y/n)? "
if [ "$REPLY" == "y" ]; then
    ssh $ossuser_qnx@$hostname -t $commands 2>&1 | tee ${NAME_LOGFILE}
    echo "====="
    scp $ossuser_qnx@$hostname:/home/$ossuser_qnx/opt_jsk_var_logs_${DATE}.tgz . | tee ${NAME_LOGFILE}

    IS_SUCCESS=0
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

if [ "${IS_SUCCESS}" -eq 1 ]; then
    echo "Operation unsuccessful. Send back log file: ${NAME_LOGFILE}"
fi
