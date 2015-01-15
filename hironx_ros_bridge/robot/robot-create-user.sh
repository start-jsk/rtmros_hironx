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
NEW_USER_QNX_CAPITAL='TORK'
OSS_FOLDER='/opt/jsk'
TMP_FOLDER='/tmp'

# Command that gets run on QNX. 
# - Create tork user,
# - Assign write access for tork user to /opt/jsk folder recursively.
# - 777 to /tmp. Ref: http://unix.stackexchange.com/a/71674/14968
commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;
  echo \"* If tork user already exists, exit. *\";
  echo \"* Else, create tork user, the one that can write into the folder where OSS gets installed. *\";
  test -e /home/$NEW_USER_QNX && (echo \"** Looks like the user '$NEW_USER_QNX' already exists. Installer exits. **\" && exit 0) || (echo \"** Looks like the user '$NEW_USER_QNX' does not exist. So let's create the user.\n   Keep using default values by pressing enter key, except for:\n\t(1) user name='$NEW_USER_QNX_CAPITAL'\n\t(2) password='$NEW_USER_QNX'.\n**\" && su -c 'passwd $NEW_USER_QNX');
  echo \"* If OSS folder '$OSS_FOLDER' exists (which should exist), change its owner to '$NEW_USER_QNX'. *\";
  su -c 'chown -R '$NEW_USER_QNX' $OSS_FOLDER';
  ls -lh $OSS_FOLDER;
  echo \"* Permit write access to $TMP_FOLDER folder. *\";
  chmod -R a=rwx,o+t $TMP_FOLDER
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
