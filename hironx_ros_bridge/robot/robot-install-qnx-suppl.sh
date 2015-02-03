#!/bin/bash

function usage {
    echo >&2 "usage: $0 [HOSTNAME_QNX (default:hiro014)] [USERNAME_QNX (default:hiro)] [QNX_MACADDR (default: xx:xx:xx:xx:xx:xx, which fails)] [HRPSYS_VER (default: 315.2.8)]"
    echo >&2 "          [-h|--help] print this message"
    exit 0
}

# command line parse. If the num of argument is not as expected, call usage func.
OPT=`getopt -o h -l help -- $*`
if [ $# != 4 ]; then
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
QNX_MACADDR=$3
QNX_MACADDR=${QNX_MACADDR:="xx:xx:xx:xx:xx:xx"}  # xx:xx:xx:xx:xx:xx should be treated as failure.
HRPSYS_VER=$4
HRPSYS_VER=${HRPSYS_VER:="315.2.8"} 

DATE=`date +"%Y%m%d-%H%M%S"`
TMP_FOLDER_ROOT=/tmp/HiroNXO_install
TMP_FOLDER=$TMP_FOLDER_ROOT/Suppl
NAME_LOGFILE=${TMP_FOLDER_ROOT}/robot-install-qnx-suppl_${HOSTNAME_QNX}_${DATE}.log
INSTALLER_NAME=robot-install-qnx-$QNX_MACADDR

# DESCRIPTION: Run on Ubuntu a QNX-specific binary `robot-install-qnx-%MAC_ADDR_QNX% %HOST_QNX% %USER_QNX%` 
#              (discussed at https://github.com/tork-a/delivery/issues/33#issuecomment-67120688).
# OUTPUT: Extracted hrpsys files under /tmp/$TMP_FOLDER-$DATE
#
# Steps taken:
## 1. Check if MAC registered in the binary matches with the QNX host this script is connecting to.
##ssh $USERNAME_QNX@$HOSTNAME_QNX export MACADDR_QNX_OBTAINED=`ifconfig en0 | awk '/address/ {print $NF}'`
##if [ MACADDR_QNX != MACADDR_QNX_OBTAINED ]
##then
##  echo "MAC ADDR didn't match:\n\tInput: $MACADDR_QNX\n\t"
##  exit 0
echo " * 1. Check if the installer file $INSTALLER_NAME exists and is executable." 
test -x $INSTALLER_NAME || (echo "Installer file $INSTALLER_NAME does not exist or is not executable. Returning." && exit 0)  
echo " * 2. Send the binary $INSTALLER_NAME into QNX under /tmp dir. This might take some time depending on the network."
scp ./$INSTALLER_NAME $USERNAME_QNX@$HOSTNAME_QNX:/home/$USERNAME_QNX
echo " * 3. On QNX under /tmp dir, extract binary file. This will yield opt/jsk folder under the temporary folder.*\nThen Move the extracted files to /opt/jsk."
commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;

  [ -d $TMP_FOLDER ] && rm -fr $TMP_FOLDER;
  mkdir -p $TMP_FOLDER && chmod 755 $TMP_FOLDER && cd $TMP_FOLDER;

  mv /home/$USERNAME_QNX/$INSTALLER_NAME .;
  echo \"* Run the installer $INSTALLER_NAME \"; ./$INSTALLER_NAME;
  echo \"* Now extracted files are:\" && ls;
  echo \"* Extracted files under $HOSTNAME_QNX:$TMP_FOLDER/opt/jsk: \" && ls ./opt/jsk;

  echo \"* Move files into /opt/jsk/$HRPSYS_VER. \";
  [ -d /opt/jsk/$HRPSYS_VER ] || { echo '** Directory /opt/jsk/$HRPSYS_VER does not exist. Make sure the script robot-compile-setup.sh finished without issues. Abort.'; exit 1; };
  cp -R ./opt/jsk/* /opt/jsk/${HRPSYS_VER} || { echo \"Failed to complete moving ./opt/jsk/* into /opt/jsk/$HRPSYS_VER. However, most likely files are moved. So judgement call is made, to move forward. \".; };

  echo \"* Note: Symbolic link from folders in /opt/jsk to the ones that contain specific hrpsys version. Sym-linking didn't work from the directory. *\";
  cd /opt/jsk;
  ln -sf /opt/jsk/$HRPSYS_VER/* . && { echo '===== Done. /opt/jsk is now structured as follows: ====='; ls -l /opt/jsk; rm -fr ${TMP_FOLDER_SETUP}; };

  echo \"* Files now extracted under ./opt/jsk. Remove /tmp/$TMP_FOLDER in case of the future execution. \" && rm -fr /tmp/$TMP_FOLDER;
  "
ssh $USERNAME_QNX@$HOSTNAME_QNX -t $commands 2>&1 | tee ${NAME_LOGFILE}
