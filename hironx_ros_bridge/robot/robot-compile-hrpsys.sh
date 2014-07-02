#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro014)] [userid (default:hiro)] [revision (default:latest)]"
    echo >&2 "          [-h|--help] print this message"
    exit 0
}

trap 'exit 1' ERR;

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


HOSTNAME=${1-"hiro014"}
USERID=${2-"hiro"}
HRPSYS_VERSION=${3-"315.1.10"}


DATE=`date +%Y-%m-%d`

wget https://github.com/fkanehiro/hrpsys-base/archive/${HRPSYS_VERSION}.zip -O /tmp/hrpsys-base-${HRPSYS_VERSION}.zip || echo "ERROR:: Failed to download source code"
(cd ../; tar -cvzf /tmp/hironx-robot-script-$DATE.tgz robot/Makefile robot/*.in robot/*.sav)

commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;
  echo \"* Download hrpsys *\";
  mkdir -p /tmp/hrpsys-source-${HRPSYS_VERSION}-${DATE}/src;
  cd /tmp/hrpsys-source-${HRPSYS_VERSION}-${DATE}/src;
  mv /tmp/hrpsys-base-$HRPSYS_VERSION.zip .;
  unzip -o hrpsys-base-$HRPSYS_VERSION.zip ;
  mkdir -p ../build;
  cd ../build;
  echo \"* Configure hrpsys *\";
  PATH=/opt/jsk/bin:/usr/pkg/bin:/usr/qnx650/host/qnx6/x86/usr/bin:$PATH LD_LIBRARY_PATH=/opt/jsk/lib:/usr/pkg/lib PKG_CONFIG_PATH=/opt/jsk/lib/pkgconfig:/usr/pkg/lib/pkgconfig CXX=QCC CC=qcc TVMET_DIR=/opt/jsk OPENRTM_IDL_DIR=/opt/jsk/include/OpenRTM-1.1/rtm/idl LDFLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_thread -lboost_filesystem\" cmake ../src/hrpsys-base-${HRPSYS_VERSION} -DLAPACK_LIBRARY_DIRS=/opt/jsk/lib -DLAPACK_INCLUDE_DIRS=/opt/jsk/include -DOPENRTM_DIR=/opt/jsk -DOPT_COLLADASUPPORT=NO -DEIGEN_INCLUDE_DIR=/opt/jsk/include -DCOMPILE_JAVA_STUFF=OFF -DCMAKE_SHARED_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_thread -lboost_regex -lf2c -luuid -lsocket -Wl,-u,MAIN__\" -DCMAKE_EXE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_thread -lboost_filesystem -lboost_regex -lf2c -luuid -lsocket -Wl,-u,MAIN__\" -DCMAKE_MODULE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_thread -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_INSTALL_PREFIX=../ -DBLAS_LIBRARY=/opt/jsk/lib/libblas.so -DLAPACK_LIBRARY=/opt/jsk/lib/liblapack.so -DG2C_LIBRARY=/opt/jsk/lib/libf2c.so -DCMAKE_CXX_FLAGS=\"-I/usr/pkg/include\" -DCMAKE_INSTALL_PREFIX=/opt/jsk -DENABLE_DOXYGEN=OFF ;
  echo \"* Compile hrpsys *\";
  PATH=/opt/jsk/bin:/usr/pkg/bin:/usr/qnx650/host/qnx6/x86/usr/bin:$PATH LD_LIBRARY_PATH=/opt/jsk/lib:/usr/pkg/lib:/usr/qnx650/host/qnx6/x86/usr/lib make VERBOSE=1;
  echo \"* Install hrpsys *\";
  make install;
  echo \"* Modify codes *\";
  rm /opt/jsk/lib/libhrpIo.so;
  echo \"* Configure robot script files  *\";
  tar -xkvzf /tmp/hironx-robot-script-$DATE.tgz;
  cd robot;
  make configure;
  make install INSTALL_DIR=/opt/jsk/;
  cd /tmp;
  echo \"* make tarball *\";
  tar -czf hrpsys-${HRPSYS_VERSION}-qnx-${DATE}.tgz ./hrpsys-source-${HRPSYS_VERSION}-${DATE}/;
  "

echo "comands = $commands"
read -p "execute compile command @ $HOSTNAME (y/n)? "
if [ "$REPLY" == "y" ]; then
    scp /tmp/hrpsys-base-${HRPSYS_VERSION}.zip /tmp/hironx-robot-script-$DATE.tgz $USERID@$HOSTNAME:/tmp/
    ssh $USERID@$HOSTNAME -t $commands 2>&1 | tee /tmp/robot-compile-hrpsys-`date +"%Y%m%d-%H%M%S"`.log
    echo "====="
    echo "$ tar -xvzf /tmp/hrpsys-${HRPSYS_VERSION}-qnx-${DATE}; cd hrpsys-source-${HRPSYS_VERSION}-${DATE}/build; make install"
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

