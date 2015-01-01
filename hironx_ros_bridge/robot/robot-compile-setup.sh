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

# Detect if the target robot is Hiro or NX.
#ssh $userid@$hostname ls /opt/hiro && export IS_TARGET_HIRO=true

TMPDIR=/tmp/HiroNXO_install
URL_HIRONX_ZIPBALL=https://github.com/start-jsk/rtmros_hironx/archive/1.0.27.zip
URL_NXO_ZIPBALL_STABLE=https://github.com/tork-a/rtmros_nextage/archive/0.2.14.zip
mkdir -p ${TMPDIR}/opt/jsk/${hrpsys_version}
# Download model files.
##wget ${URL_HIRONX_ZIPBALL} -O ${TMPDIR}/rtmros_hironx.zip || echo "ERROR:: Failed to download hironx zipball."
##(cd ${TMPDIR}; unzip -o rtmros_hironx.zip)
wget ${URL_NXO_ZIPBALL_STABLE} -O ${TMPDIR}/rtmros_nextage.zip || echo "ERROR:: Failed to download nxo zipball."
(cd ${TMPDIR}; unzip -o rtmros_nextage.zip)
# Download 'base' libraries for QNX that don't be affected by the hrpsys version
wget 'https://docs.google.com/uc?authuser=0&id=0B5hXrFUpyR2iZS0tQlFyXzhjaGc&export=download' -O ${TMPDIR}/opt-jsk-base.tgz || echo "ERROR:: Failed to download opt-jsk-base.tgz."

(cd ${TMPDIR}; tar -xvzf opt-jsk-base.tgz)
# The tarball above (opt-jsk-base.tgz) contains `include` and `lib` folders, only. So from here we'll create other folders.
mkdir -p ${TMPDIR}/opt/jsk/${hrpsys_version}/etc
##mv ${TMPDIR}/*/hironx_ros_bridge/ ${TMPDIR}/opt/jsk/${hrpsys_version}/etc/HIRONX
# zipball retrieved by URL_NXO_ZIPBALL_STABLE yields random folder name. So skip it by asterisk.
mv ${TMPDIR}/*/nextage_description/ ${TMPDIR}/opt/jsk/${hrpsys_version}/etc/NEXTAGE
# VRML model file of Hiro is not opensourced yet. It should be found in /opt/jsk/etc/HIRONX in each robot.
#mv ${TMPDIR}/opt/jsk/${hrpsys_version}/etc/HIRONX/models ${TMPDIR}/opt/jsk/${hrpsys_version}/etc/HIRONX/model
ssh $userid@$hostname ls /opt/hiro && (export IS_TARGET_HIRO="true"; ssh $userid@$hostname "tar cfvz /tmp/hiro_wrl.tgz /opt/jsk/etc/HIRONX;"; scp $userid@$hostname:/tmp/hiro_wrl.tgz /tmp/hiro_wrl.tgz; cd ${TMPDIR} && tar xfvz /tmp/hiro_wrl.tgz ${TMPDIR}/opt/jsk/${hrpsys_version}/etc/; ls ${TMPDIR}) || echo "The robot you're dealing with now is NEXTAGE."
##HIRO_COPY_MODEL="echo 'It is Hiro, so use existing model files.'; cp -R /opt/jsk/$HRPSYS_PREV_INSTALLED/etc/HIRONX /opt/jsk/$hrpsys_version/etc;" || echo "The robot you're dealing with now is NEXTAGE."
# Folder names from nextage_description and the one used internal to QNX is different. See https://github.com/start-jsk/rtmros_hironx/issues/160#issuecomment-48572336
mv ${TMPDIR}/opt/jsk/${hrpsys_version}/etc/NEXTAGE/models ${TMPDIR}/opt/jsk/${hrpsys_version}/etc/NEXTAGE/model
tar -C ${TMPDIR} -cvzf ${TMPDIR}/opt-jsk-base-model.tgz ./opt/jsk/${hrpsys_version}/

# Get the version of hrpsys that is working inside of QNX.
scp $userid@$hostname:/opt/jsk/lib/RobotHardware.so /tmp
HRPSYS_PREV_INSTALLED=$(strings /tmp/RobotHardware.so  | grep ^[0-9]*\\.[0-9]*\\.[0-9]*$)

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
  echo \"* So move it aside with hrpsys version and the timestamp as a part of dir name (eg. /opt/jsk/315.2.0_20141214). *\";
  test -L /opt/jsk && su -c 'rm -fr /opt/jsk' || (mkdir /tmp/$HRPSYS_PREV_INSTALLED && su -c 'mv /opt/jsk/* /tmp/$HRPSYS_PREV_INSTALLED'; echo 'ls /opt/jsk' && ls /opt/jsk && su -c 'mv /tmp/$HRPSYS_PREV_INSTALLED /opt/jsk');
  echo \"* setup /opt/jsk/$hrpsys_version directory *\";
  cd /;
  su -c 'tar -xvzf /tmp/opt-jsk-base-model.tgz';
  echo \"* Create supplemental folder '/opt/jsk/$hrpsys_version/var/log' *\";
  su -c 'mkdir -p /opt/jsk/$hrpsys_version/var/log';
  echo \"* Symbolic link from folders in /opt/jsk to the ones that contain specific hrpsys version. Sym-linking didn't work from the directory. *\";
  cd /opt/jsk;
  su -c 'ln -sf /opt/jsk/$hrpsys_version/* .';
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
