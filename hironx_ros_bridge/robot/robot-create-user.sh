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
TMP_FOLDER='/tmp'

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
  echo \"* If tork user already exists, exit. *\";
  echo \"* Else, create tork user, the one that can write into the folder where OSS gets installed. *\";
  test -e /home/$NEW_USER_QNX && { echo \"** Looks like the user '$NEW_USER_QNX' already exists. Installer exits. **\" && exit 0; } || { echo \"** Looks like the user '$NEW_USER_QNX' does not exist. So let's create the user.\n   Keep using default values by pressing enter key, except for:\n\t(1) user name='$NEW_USER_QNX_CAPITAL'\n\t(2) password='$NEW_USER_QNX'.\n**\" && su -c 'passwd $NEW_USER_QNX'; };

  echo \"* If oss group already exists, exit. Else, create one. If creation fails, abort. *\";
  id -g ${NEW_GROUPS_QNX} && echo 'Group ${NEW_GROUPS_QNX} already exists.' || { { echo 'Group ${NEW_GROUPS_QNX} not exist, create one.'; su -c 'echo 'oss:x:${NEW_GROUP_ID_QNX}:${NEW_USER_QNX}' >> /etc/group' && { echo 'Group added to /etc/group'; cat /etc/group; } || { echo 'Something happened while creating a new group. Check if groupd id ${NEW_GROUP_ID_QNX} is not taken in /etc/group. Abort.'; exit 1; }; }
; };

  echo \"* If OSS folder '$OSS_FOLDER' exists (which should exist), change its group to '${NEW_GROUPS_QNX}'. *\";
  su -c 'chgrp -R ${NEW_GROUPS_QNX} $OSS_FOLDER';
  ls -lh $OSS_FOLDER;

  echo \"* Change group of ${TMP_FOLDER} folder to ${NEW_GROUPS_QNX}. *\";
  su -c 'chgrp -R ${NEW_GROUPS_QNX} ${TMP_FOLDER}';  
  ls -lh / | grep tmp;
  "

read -p "Run the command @ $hostname (y/n)? "
if [ "$REPLY" == "y" ]; then
    ssh $rootuser_qnx@$hostname -t $commands 2>&1 | tee /tmp/robot-create-user-`date +"%Y%m%d-%H%M%S"`.log
    echo "====="
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

rm -fr ${TMP_FOLDER}
