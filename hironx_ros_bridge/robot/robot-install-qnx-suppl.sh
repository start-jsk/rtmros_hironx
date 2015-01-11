#!/bin/bash

function usage {
    echo >&2 "usage: $0 [HOSTNAME_QNX (default:hiro014)] [USERNAME_QNX (default:hiro)] [HRPSYS_VER (default: 315.2.8)]"
    echo >&2 "          [-h|--help] print this message"
    exit 0
}

# command line parse. If the num of argument is not as expected, call usage func.
OPT=`getopt -o h -l help -- $*`
if [ $# != 3 ]; then
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

INSTALLER_NAME=robot-install-qnx

DATE=`date +%Y-%m-%d`

# DESCRIPTION: Run on Ubuntu a QNX-specific binary `robot-install-qnx-%MAC_ADDR_QNX% %HOST_QNX% %USER_QNX%` 
#              (discussed at https://github.com/tork-a/delivery/issues/33#issuecomment-67120688).
# OUTPUT: Extracted hrpsys files under /tmp/oss-install-$DATE-work
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
scp ./$INSTALLER_NAME $USERNAME_QNX@$HOSTNAME_QNX:/tmp/oss-install-$DATE
echo " * 3. On QNX under /tmp dir, extract binary file. This will yield opt/jsk folder under the temporary folder.*\nThen Move the extracted files to /opt/jsk."
commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;
  mkdir /tmp/oss-install-$DATE-work;
  cd /tmp/oss-install-$DATE-work;
  mv ../oss-install-$DATE .;
  chmod 755 ./oss-install-$DATE;
  echo \"* Run oss-install-$DATE \";
  ./oss-install-$DATE;
  echo \"* Now extracted files are:\" && ls;
  echo \"* Extracted files under ./opt/jsk: \" && ls ./opt/jsk;
  echo \"* Files now extracted under ./opt/jsk. Remove /tmp/oss-install-$DATE-work in case of the future execution. \" && rm -fr /tmp/oss-install-$DATE-work;
  "
backedup="    
  su -c 'mv ./opt/jsk/* /opt/jsk/%HRPSYS_VER%';
  su -c 'ln -sf /opt/jsk/$HRPSYS_VER/* .';
  "
ssh $USERNAME_QNX@$HOSTNAME_QNX -t $commands 2>&1 | tee /tmp/robot-install-qnx-suppl-`date +"%Y%m%d-%H%M%S"`.log

rm -fr ${TMPDIR}
