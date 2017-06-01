#!/bin/bash

function usage {
    echo >&2 "usage: $1 [hostname (default:nextage)]"
    echo >&2 "       $2 [ossuser_qnx (default:tork)]"
    echo >&2 "       $3 [date_specifier (default:none. Format: yyyymmdd, or it can be partial, e.g. yyyymm. Searches the files whose name matches the given string.)]"
    echo >&2 "          [-h|--help] Print help message."
    exit 0
}

# command line parse. If the num of argument is not as expected, call usage func.
OPT=`getopt -o h -l help -- $*`
if [ $# -lt 1 ]; then
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
hostname=${hostname:="nextage"} 
ossuser_qnx=$2
ossuser_qnx=${ossuser_qnx:="tork"}
date_specifier=$3
date_specifier=${date_specifier:=""}  # By default this isn't set, so fetch files for all dates.
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
  tar cfvz opt_jsk_var_logs_${DATE}.tgz ${OSS_FOLDER_LOG}/*${date_specifier}* || echo '* Failed to create tarball *';

  echo '* Exitting ssh session to QNX. *';
  exit;
  "

IS_SUCCESS=1

echo "** You can open the script and check inside **"
echo "** if you are unsure about the content of this script. **"
echo "** You can also find the same script source online at **"
echo "** https://github.com/start-jsk/rtmros_hironx/blob/indigo-devel/hironx_ros_bridge/robot/qnx_fetch_log.sh **"
echo "** Report if there's any issue for this script at https://github.com/start-jsk/rtmros_hironx/issues **"

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
