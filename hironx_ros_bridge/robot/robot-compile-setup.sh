#!/bin/bash

function usage {
    echo >&2 "usage: $0 [HOSTNAME_QNX (default:hiro014)] [USERNAME_QNX (default:hiro)] [HRPSYS_VER (default:315.2.8)]"
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

HOSTNAME_QNX=$1
HOSTNAME_QNX=${HOSTNAME_QNX:="hiro014"} 
USERNAME_QNX=$2
USERNAME_QNX=${USERNAME_QNX:="hiro"} 
HRPSYS_VER=$3
HRPSYS_VER=${HRPSYS_VER:="315.2.8"} 

DATE=`date +"%Y%m%d-%H%M%S"`
TMP_FOLDER=/tmp/HiroNXO_install
TMP_FOLDER_SETUP=$TMP_FOLDER/Setup
NAME_LOGFILE=${TMP_FOLDER}/robot-compile-setup_${HOSTNAME_QNX}_${DATE}.log
TARBALL_OSS_COMMON=opt-jsk-base.tgz      # Base, common files of OSS controller
TARBALL_MODELS=opt-jsk-base-model.tgz  # model files of both Hiro and NXO.
OSS_FOLDER='/opt/jsk'

URL_HIRONX_ZIPBALL=https://github.com/start-jsk/rtmros_hironx/archive/1.0.27.zip
URL_NXO_ZIPBALL_STABLE=https://github.com/tork-a/rtmros_nextage/archive/0.2.14.zip
[ -d ${TMP_FOLDER_SETUP} ] && { echo 'In case working folder from previous execution remains, remove and recreate it.'; rm -fr ${TMP_FOLDER_SETUP}; }; mkdir -p ${TMP_FOLDER_SETUP}/opt/jsk/${HRPSYS_VER}
# Download model files.
##wget ${URL_HIRONX_ZIPBALL} -O ${TMP_FOLDER_SETUP}/rtmros_hironx.zip || echo "ERROR:: Failed to download hironx zipball."
##(cd ${TMP_FOLDER_SETUP}; unzip -o rtmros_hironx.zip)
wget ${URL_NXO_ZIPBALL_STABLE} -O ${TMP_FOLDER_SETUP}/rtmros_nextage.zip || echo "ERROR:: Failed to download nxo zipball."
{ cd ${TMP_FOLDER_SETUP}; unzip -o rtmros_nextage.zip; }
# Download 'base' libraries for QNX that don't be affected by the hrpsys version
wget 'https://docs.google.com/uc?authuser=0&id=0B5hXrFUpyR2iZS0tQlFyXzhjaGc&export=download' -O ${TMP_FOLDER_SETUP}/${TARBALL_OSS_COMMON} || { echo "ERROR:: Failed to download ${TARBALL_OSS_COMMON}. Abort."; exit 1; }
# Move files in the tarball above to ${HRPSYS_VER}. TARBALL_OSS_COMMON extracts to ./opt/jsk/* directories.
tar -C ${TMP_FOLDER_SETUP} -xvzf ${TARBALL_OSS_COMMON}; { mv ./opt/jsk/include ${TMP_FOLDER_SETUP}/opt/jsk/${HRPSYS_VER}; mv ./opt/jsk/lib ${TMP_FOLDER_SETUP}/opt/jsk/${HRPSYS_VER}; } || { echo "ERROR happened while moving ${TARBALL_OSS_COMMON} contents. Abort."; exit 1; }

# The tarball above (${TARBALL_OSS_COMMON}) contains `include` and `lib` folders, only. So from here we'll create other folders.
mkdir -p ${TMP_FOLDER_SETUP}/opt/jsk/${HRPSYS_VER}/etc
##mv ${TMP_FOLDER_SETUP}/*/hironx_ros_bridge/ ${TMP_FOLDER_SETUP}/opt/jsk/${HRPSYS_VER}/etc/HIRONX
# zipball retrieved by URL_NXO_ZIPBALL_STABLE yields random folder name. So skip it by asterisk.
mv ${TMP_FOLDER_SETUP}/*/nextage_description/ ${TMP_FOLDER_SETUP}/opt/jsk/${HRPSYS_VER}/etc/NEXTAGE
# VRML model file of Hiro is not opensourced yet. It should be found in /opt/jsk/etc/HIRONX in each robot.
# Detect if the target robot is Hiro or NX.
# If Hiro, fetch tarball of /opt/hiro to the ${TMP_FOLDER_SETUP}/hiro_wrl.tgz on the local machine, then extract the content.
IS_QNX_HIRO=is_qnx_hiro
commands_check_hiro="
  ls /opt/hiro && { echo 'Ok, the robot on QNX is Hiro.' && tar cfvz $TMP_FOLDER/hiro_wrl.tgz /opt/jsk/etc/HIRONX && true; } || { echo 'No, this robot on QNX is NXO.'; false; } && echo 'true' > $TMP_FOLDER/$IS_QNX_HIRO;
  "
{ ssh $USERNAME_QNX@$HOSTNAME_QNX -t $commands_check_hiro; echo "----$commands_check_hiro is over.----"; } 2>&1 | tee -a ${NAME_LOGFILE}
#    Then get the tarball from Hiro.
if [ -e $TMP_FOLDER/$IS_QNX_HIRO ];
then
  { scp $USERNAME_QNX@$HOSTNAME_QNX:${TMP_FOLDER}/hiro_wrl.tgz ${TMP_FOLDER_SETUP}/hiro_wrl.tgz; echo 'Create a tarball for hiro model files.' && tar -C ${TMP_FOLDER_SETUP} xfvz ${TMP_FOLDER_SETUP}/hiro_wrl.tgz ${TMP_FOLDER_SETUP}/opt/jsk/${HRPSYS_VER}/etc/; } && { echo 'Model files of Hiro are retrieved from robot.'; ls ${TMP_FOLDER_SETUP}; } || echo "The robot you're dealing with now is NEXTAGE.";
else
  echo 'Nothing done for NXO here.';
fi
# Folder names from nextage_description and the one used internal to QNX is different. See https://github.com/start-jsk/rtmros_hironx/issues/160#issuecomment-48572336
mv ${TMP_FOLDER_SETUP}/opt/jsk/${HRPSYS_VER}/etc/NEXTAGE/models ${TMP_FOLDER_SETUP}/opt/jsk/${HRPSYS_VER}/etc/NEXTAGE/model
echo "Creating ${TMP_FOLDER_SETUP}/${TARBALL_MODELS}"; tar -C ${TMP_FOLDER_SETUP}/opt/jsk -cvzf ${TARBALL_MODELS} ${HRPSYS_VER}

# Get the version of hrpsys that is working inside of QNX.
scp $USERNAME_QNX@$HOSTNAME_QNX:/opt/jsk/lib/RobotHardware.so ${TMP_FOLDER_SETUP}
HRPSYS_PREV_INSTALLED=$(strings ${TMP_FOLDER_SETUP}/RobotHardware.so | grep ^[0-9]*\\.[0-9]*\\.[0-9]*$)
if [ -z ${HRPSYS_PREV_INSTALLED} ]; then HRPSYS_PREV_INSTALLED=315.1.7; fi # getting version above might fail with older hrpsys. Add dummy version. See https://github.com/fkanehiro/hrpsys-base/issues/423#issuecomment-71299696

# Command that gets run on QNX. 
# Assumption: NO /opt/jsk directory exists on QNX.
# Writing into new directory that's created by root requires root privilege.
# TODO: Currently backing up the entire /opt/jsk so that in the versioned folder previous versions get contained. This needs to be fixed.
#       e.g.
#            315.2.0
#            |- 315.1.8
#            |- include
#            |- lib
commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;

  mkdir -p ${TMP_FOLDER_SETUP} && { echo '** Foder ${TMP_FOLDER_SETUP} is created.'; mv /home/$USERNAME_QNX/${TARBALL_MODELS} ${TMP_FOLDER_SETUP}; } || echo \"Failed to either create folder ${TMP_FOLDER_SETUP}, or mv failed 'mv /home/$USERNAME_QNX/${TARBALL_MODELS} ${TMP_FOLDER_SETUP}' \";
  echo \"* If /opt/jsk is a symlink, remove it (it'll be generated later within this script. *\";
  echo \"* Else if /opt/jsk is a directory, not a symlink, it's a folder containing OSS controller libraries. *\";
  echo \"* So move it aside with hrpsys version and the timestamp as a part of dir name (eg. /opt/jsk/315.2.0_20141214). *\";
  test -L /opt/jsk && rm -fr /opt/jsk || mkdir -p ${TMP_FOLDER_SETUP}/${HRPSYS_PREV_INSTALLED} && { mv /opt/jsk/* ${TMP_FOLDER_SETUP}/${HRPSYS_PREV_INSTALLED} && { echo '* ls /opt/jsk'; ls -l /opt/jsk && mv ${TMP_FOLDER_SETUP}/$HRPSYS_PREV_INSTALLED /opt/jsk || { echo 'ls /opt/jsk does not exist. Abort.'; exit 1; } } || { echo 'Move ${TMP_FOLDER_SETUP}/$HRPSYS_PREV_INSTALLED to /opt/jsk failed somehow. Abort'; exit 1; } || echo '** Something did not go well during moving previous version of hrpsys back to /opt/jsk. Make sure that the groups setting is oss in /opt/jsk/'; } || { echo '* Failed to create ${TMP_FOLDER_SETUP}/${HRPSYS_PREV_INSTALLED}'; exit 1; };

  echo \"* setup /opt/jsk/$HRPSYS_VER directory *\"; cd /opt/jsk || { echo '** cd /opt/jsk failed. Abort.'; exit 1; };
  tar -xvzf ${TMP_FOLDER_SETUP}/${TARBALL_MODELS} || { echo \"** tar -xvzf ${TMP_FOLDER_SETUP}/${TARBALL_MODELS} failed.\"; exit 1; };
  echo \"* Create supplemental folder '/opt/jsk/$HRPSYS_VER/var/log' *\"; mkdir -p /opt/jsk/$HRPSYS_VER/var/log;
  su -c 'chmod -R 775 ${OSS_FOLDER}' && { echo '* chmod ${OSS_FOLDER} successful.'; ls -lh ${OSS_FOLDER}; } || { echo '** chmod -R 775 ${OSS_FOLDER} failed. abort.'; exit 1; };
  echo '*****************************';
  echo '*** Operation successful. ***';
  echo '*****************************';
  "

echo "Necessary files are prepared on your host and now ready to run these commands inside the robot = $commands"
read -p "execute installation command @ $HOSTNAME_QNX (y/n)? "
if [ "$REPLY" == "y" ]; then
    echo "scp ${TMP_FOLDER_SETUP}/${TARBALL_MODELS} into $HOSTNAME_QNX"
    { scp ${TMP_FOLDER_SETUP}/${TARBALL_MODELS} $USERNAME_QNX@$HOSTNAME_QNX:/home/$USERNAME_QNX && ssh $USERNAME_QNX@$HOSTNAME_QNX -t $commands 2>&1 | tee -a ${NAME_LOGFILE}; } || echo "Aborted."
    echo "====="
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

rm -fr ${TMP_FOLDER_SETUP}
