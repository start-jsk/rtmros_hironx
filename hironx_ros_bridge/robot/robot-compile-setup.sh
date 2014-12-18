#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro014)] [username (default:hiro)] [hrpsys_version (default:315.2.8)]"
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
hrpsys_version=$3
hrpsys_version=${hrpsys_version:="315.2.8"} 

DATE=`date +%Y-%m-%d`

TMPDIR=/tmp/HOGE
URL_NXO_LATEST_ZIPBALL=https://github.com/tork-a/rtmros_nextage/archive/0.2.14.zip
mkdir -p ${TMPDIR}/opt/jsk_${hrpsys_version}
# Download NEXTAGE model files.
wget ${URL_NXO_LATEST_ZIPBALL} -O ${TMPDIR}/rtmros_nextage.zip || echo "ERROR:: Failed to download source code"
# Download 'base' libraries for QNX that don't be affected by the hrpsys version  
wget 'https://docs.google.com/uc?authuser=0&id=0B5hXrFUpyR2iZS0tQlFyXzhjaGc&export=download' -O ${TMPDIR}/opt-jsk-base.tgz || echo "ERROR:: Failed to download source code"

(cd ${TMPDIR}; tar -xvzf opt-jsk-base.tgz)
(cd ${TMPDIR}; unzip -o rtmros_nextage.zip)
mkdir -p ${TMPDIR}/opt/jsk_${hrpsys_version}/etc
# zipball retrieved by URL_NXO_LATEST_ZIPBALL yields random folder name. So skip it by asterisk.
mv ${TMPDIR}/*/nextage_description/ ${TMPDIR}/opt/jsk_${hrpsys_version}/etc/HIRONX 
mv ${TMPDIR}/opt/jsk_${hrpsys_version}/etc/HIRONX/models ${TMPDIR}/opt/jsk_${hrpsys_version}/etc/HIRONX/model
tar -C ${TMPDIR} -cvzf ${TMPDIR}/opt-jsk-base-model.tgz ./opt/jsk_${hrpsys_version}/

# Command that gets run on QNX. 
# Assumption: NO /opt/jsk directory exists on QNX.
# Writing into new directory that's created by root requires root privilege.
commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;
  echo \"* If /opt/jsk is a symlink, remove it (it'll be generated later within this script. *\";
  echo \"* Else if /opt/jsk is a directory, not a symlink, it's a folder containing OSS controller libraries. *\";
  echo \"* So move it aside with hrpsys version and the timestamp as a part of dir name (eg. /opt/jsk_315.2.0_20141214). *\";
  test -L /opt/jsk && su -c 'rm -fr /opt/jsk' || su -c 'mv /opt/jsk /opt/jsk_$(strings RobotHardware.so | grep ^[0-9]*\\.[0-9]*\\.[0-9]*$)_$(date +%Y%m%d)'
  echo \"* setup /opt/jsk_$hrpsys_version directory *\";
  cd /;
  su -c 'tar -xvzf /tmp/opt-jsk-base-model.tgz';
  echo \"* Create supplemental folder '/opt/jsk_$hrpsys_version/var/log' *\";
  su -c 'mkdir -p /opt/jsk_$hrpsys_version/var/log';
  echo \"* Symbolic link from /opt/jsk to the one that contains specific hrpsys version. *\";
  su -c 'ln -sf /opt/jsk_$hrpsys_version /opt/jsk';
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
