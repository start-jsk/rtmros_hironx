#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro014)] [username (default:hiro)]"
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

hostname=$1
hostname=${hostname:="hiro014"} 
userid=$2
userid=${userid:="hiro"} 

DATE=`date +%Y-%m-%d`

TMPDIR=/tmp/HOGE
URL_NXO_LATEST_ZIPBALL=https://github.com/tork-a/rtmros_nextage/archive/0.2.14.zip
mkdir -p ${TMPDIR}/opt/jsk
wget ${URL_NXO_LATEST_ZIPBALL} -O ${TMPDIR}/rtmros_nextage.zip || echo "ERROR:: Failed to download source code"
wget 'https://docs.google.com/uc?authuser=0&id=0B5hXrFUpyR2iZS0tQlFyXzhjaGc&export=download' -O ${TMPDIR}/opt-jsk-base.tgz || echo "ERROR:: Failed to download source code"

(cd ${TMPDIR}; tar -xvzf opt-jsk-base.tgz)
(cd ${TMPDIR}; unzip -o rtmros_nextage.zip)
mkdir -p ${TMPDIR}/opt/jsk/etc
# zipball retrieved by URL_NXO_LATEST_ZIPBALL yields random folder name. So skip it by asterisk.
mv ${TMPDIR}/*/nextage_description/ ${TMPDIR}/opt/jsk/etc/HIRONX 
mv ${TMPDIR}/opt/jsk/etc/HIRONX/models ${TMPDIR}/opt/jsk/etc/HIRONX/model
tar -C ${TMPDIR} -cvzf ${TMPDIR}/opt-jsk-base-model.tgz ./opt/jsk/

commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;
  echo \"* setup /opt/jsk directory *\";
  cd /;
  tar -xvzf /tmp/opt-jsk-base-model.tgz;
  mkdir -p /opt/jsk/var/log
  "

echo "Necessary files are prepared on your host and now ready to run these commands inside the robot = $commands"
read -p "execute compile command @ $hostname (y/n)? "
if [ "$REPLY" == "y" ]; then
    scp ${TMPDIR}/opt-jsk-base-model.tgz $userid@$hostname:/tmp/
    ssh $userid@$hostname -t $commands 2>&1 | tee /tmp/robot-compile-setup-`date +"%Y%m%d-%H%M%S"`.log
    echo "====="
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

rm -fr ${TMPDIR}
