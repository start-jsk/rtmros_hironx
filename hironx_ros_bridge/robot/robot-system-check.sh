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

TMPDIR=tmp.$$
commands="
  . ~/.profile;
  env;
  ls -al /tmp/$TMPDIR/robot-system-check-base;
  /tmp/$TMPDIR/robot-system-check-base;
  rm -fr /tmp/$TMPDIR
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

getent ahosts $hostname || (echo -e "-- [ERROR] Could find IP address/Host name for $hostname"; exit 1 ; )


echo ";; Copying check script to $userid@$hostname:$TMPDIR"
ssh  $userid@$hostname "touch /opt/jsk/.checked"
ssh  $userid@$hostname "mkdir -p /tmp/$TMPDIR"
scp  ./check/robot-system-check-base $userid@$hostname:/tmp/$TMPDIR/
echo ";; Execute check scripts"
ssh $userid@$hostname -t $commands 2>&1 | tee robot-system-check-$hostname.log

echo -e ";;\n;;\n;; Done check scripts, please check robot-system-check-$hostname.log file\n;;\n;;\n"


echo "hit any key to exit"; read DUMMY;
