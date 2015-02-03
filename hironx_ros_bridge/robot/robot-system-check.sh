#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:qnx)] [username (default:tork)]"
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

TMP_FOLDER=tmp.$$
commands="
  . ~/.profile;
  env;
  trap 'RET=\$?; exit \${RET}' 0;
  ls -al /tmp/$TMP_FOLDER/robot-system-check-base;
  /tmp/$TMP_FOLDER/robot-system-check-base;
"

hostname=$1
hostname=${hostname:="qnx"}
userid=$2
userid=${userid:="tork"}

trap 'echo "hit any key to exit"; read DUMMY; exit 1' ERR;

DISTRO=`lsb_release  -cs`
echo ";; check Ubuntu version $DISTRO"
[ $DISTRO == "precise" ] || (echo -e "-- [ERROR] Wrong ubuntu version\n`lsb_release -a`"; exit 1 ; )

ping -c1 $hostname || (echo -e "-- [ERROR] Could not connect to $hostname"; exit 1 ; )

getent ahosts $hostname || (echo -e "-- [ERROR] Could not find IP address/Host name for $hostname"; exit 1 ; )

# http://www.linuxquestions.org/questions/programming-9/multiple-scp-ssh-in-one-bash-script-836919/
# http://unix.stackexchange.com/questions/14270/get-exit-status-of-process-thats-piped-to-another
echo ";; Copying check script to $userid@$hostname:$TMP_FOLDER"
ssh -NfM -o 'ControlPath=~/.ssh/%r@%h:%p.conn' "$userid@$hostname"
ssh -o 'ControlPath=~/.ssh/%r@%h:%p.conn'  $userid@$hostname "mkdir -p /tmp/$TMP_FOLDER"
scp -o 'ControlPath=~/.ssh/%r@%h:%p.conn'  ./robot-system-check-base $userid@$hostname:/tmp/$TMP_FOLDER/
echo ";; Execute check scripts"
ssh -o 'ControlPath=~/.ssh/%r@%h:%p.conn'  $userid@$hostname -t $commands 2>&1 | tee robot-system-check-$hostname.log
RESULT=${PIPESTATUS[0]}
ssh -o 'ControlPath=~/.ssh/%r@%h:%p.conn' -O exit "$userid@$hostname"

echo $RESULT
echo -e ";;\n;;\n;; Done check scripts, please check robot-system-check-$hostname.log file\n;;\n;;\n"

if [ $RESULT == 0 ]; then
    echo -e ";; RESULT: SUCCEED"
else
    echo -e ";; RESULT: FAILED"
fi



echo "hit any key to exit"; read DUMMY;

exit $RESULT

