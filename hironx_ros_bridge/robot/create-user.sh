#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro014)] [rootuser_qnx (default:hiro)]"
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
hostname=${hostname:="hiro014"} 
rootuser_qnx=$2
rootuser_qnx=${rootuser_qnx:="hiro"}
NEW_USER_QNX='tork'
NEW_GROUPS_QNX='oss'
NEW_GROUP_ID_QNX='130'
NEW_USER_QNX_CAPITAL='TORK'
OSS_FOLDER='/opt/jsk'
DATE=`date +"%Y%m%d-%H%M%S"`
TMP_FOLDER_ROOT=/tmp/HiroNXO_install
NAME_LOGFILE=${TMP_FOLDER_ROOT}/create-user_${hostname}_${DATE}.log

mkdir ${TMP_FOLDER_ROOT}

# Command that gets run on QNX. 
# - Create tork user,
# - Create oss group. Add tork to oss group.
# - Assign write access for oss group to /opt/jsk folder recursively.
# - 775 to /tmp. Ref: http://unix.stackexchange.com/a/71674/14968. Using group is better because it reduces risk of /opt/jsk folder to be randomely edited, particularly by accident.
commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;

  echo '* If tork user already exists, exit. *';
  echo '* Else, create tork user, the one that can write into the folder where OSS gets installed. *';
  test -e /home/$NEW_USER_QNX && echo '** Looks like the user $NEW_USER_QNX already exists. **' || { echo '** Looks like the user $NEW_USER_QNX does not exist. So let us create the user.\n   Keep using default values by pressing enter key, except for:\n\t* user name=$NEW_USER_QNX_CAPITAL\n\t* password=$NEW_USER_QNX.\n**' && su -c 'passwd $NEW_USER_QNX' || { echo 'Creation of user ${NEW_USER_QNX} failed. See the error message that should be printed.'; exit 1; }; };

  echo '* If oss group already exists, exit. Else, create one. If creation fails, abort.\n  NOTE: Group existence checking is not perfect. *';
  egrep -i ${NEW_GROUPS_QNX} /etc/group | grep ${NEW_GROUP_ID_QNX} && echo '** Group ${NEW_GROUPS_QNX} already exists.' || { { echo 'Group ${NEW_GROUPS_QNX} not exist, create one.\n* Enter password of $rootuser_qnx user'; su -c 'echo 'oss:x:${NEW_GROUP_ID_QNX}:${NEW_USER_QNX}' >> /etc/group' && { echo 'Group added to /etc/group'; cat /etc/group; } || { echo 'Something happened while creating a new group. Check if groupd id ${NEW_GROUP_ID_QNX} is not taken in /etc/group. Abort.'; exit 1; }; }; };

  echo '* If OSS folder '$OSS_FOLDER' exists (which should exist), change its group to '${NEW_GROUPS_QNX}'. *';
  echo '* Enter password of $rootuser_qnx user *';
  su -c 'chgrp -R ${NEW_GROUPS_QNX} $OSS_FOLDER' && ls -lh $OSS_FOLDER || { echo 'Folder $OSS_FOLDER was not be able to be assinged the group ${NEW_GROUPS_QNX}.'; exit 1; };

  echo '* Change group of /tmp folder to ${NEW_GROUPS_QNX}. *';
  echo '* Enter password of $rootuser_qnx user *';
  su -c 'chgrp -R ${NEW_GROUPS_QNX} /tmp' && ls -lh / | grep tmp || { echo 'Change group failed.'; exit 1; };

  echo '* Assign writable to group at ${OSS_FOLDER} folder *';
  echo '* Enter password of $rootuser_qnx user *';
  su -c 'chmod -R 775 ${OSS_FOLDER}' && ls -lh ${OSS_FOLDER} || { echo 'chmod failed.'; exit 1; };

  echo '*****************************';
  echo '*** Operation successful. ***';
  echo '*****************************';
  "

IS_SUCCESS=1

read -p "Run the command @ $hostname (y/n)? "
if [ "$REPLY" == "y" ]; then
    ssh $rootuser_qnx@$hostname -t $commands 2>&1 | tee ${NAME_LOGFILE}
    echo "====="
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
