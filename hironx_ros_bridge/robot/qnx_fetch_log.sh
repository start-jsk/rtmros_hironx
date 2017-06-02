#!/bin/bash

function usage {
    echo >&2 "usage: $1 [hostname (default:nextage)]"
    echo >&2 "       $2 [ossuser_qnx (default:tork)]"
    echo >&2 "       $3 [do_archive (default:1): If a string 'archive' is passed, remove all raw log files to save the disk space (date specified in 'date_after' will be ignored). Existing zip archives will not be deleted.]"
    echo >&2 "       $4 [date_after (default:none. Format: yyyy-mm-dd or mmddyyyy): Do not operate on files modified prior to the specified date.]"
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
do_archive=$3
do_archive=${do_archive:="1"}  # 1 is set by default. Actually anything other than "archive" is ignored.
date_after=$4
date_after=${date_after:="1970-01-01"}
OSS_FOLDER='/opt/jsk'
OSS_FOLDER_LOG=${OSS_FOLDER}/var/log
DATE=`date +"%Y%m%d-%H%M%S"`
TMP_FOLDER_ROOT=/tmp/HiroNXO_Log
NAME_LOGFILE_SCRIPT=${TMP_FOLDER_ROOT}/qnx_fetch_log_${hostname}_${DATE}.log
NAME_LOGARCHIVE_FILE=opt_jsk_var_logs_${DATE}.zip
NAME_LOGARCHIVE=/tmp/${NAME_LOGARCHIVE_FILE}

mkdir ${TMP_FOLDER_ROOT}

COMMAND_ARCHIVE=''
if [ ${do_archive} == "archive" ]; then
  echo "* Log files will be archived.";
  COMMAND_ARCHIVE="rm ${OSS_FOLDER_LOG}/*.log; mv ${NAME_LOGARCHIVE} ${OSS_FOLDER_LOG}";
  NAME_LOGARCHIVE=${OSS_FOLDER_LOG}/${NAME_LOGARCHIVE_FILE}
fi
  
# Command that gets run on QNX. 
# - (Assumes remotely logged in to qnx via ssh)
# - Create tarball of /opt/jsk/var/log/* at /home/tork>
# - Log out of QNX
# - scp the tarball
commands="
  . ~/.profile || echo '~/.profile does not exist. Move on anyway.';
  env;
  trap 'exit 1' ERR;
  set -x;

  echo '* Create tarball of ${OSS_FOLDER_LOG} *';
  zip -t ${date_after} ${NAME_LOGARCHIVE} ${OSS_FOLDER_LOG}/*.log;
  eval ${COMMAND_ARCHIVE};

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
    ssh $ossuser_qnx@$hostname -t $commands 2>&1 | tee ${NAME_LOGFILE_SCRIPT}
    echo "====="
    scp $ossuser_qnx@$hostname:${NAME_LOGARCHIVE} . | tee ${NAME_LOGFILE_SCRIPT}

    IS_SUCCESS=0
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

if [ "${IS_SUCCESS}" -eq 1 ]; then
    echo "Operation unsuccessful. Send back log file: ${NAME_LOGFILE_SCRIPT}"
fi
