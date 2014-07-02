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

DATE=`date +%Y-%m-%d`
OPENHRP3_VERSION=3.1.5

commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;
  echo \"* Download openhrp3 *\";
  mkdir -p /tmp/openhrp3-source-$DATE/src;
  cd /tmp/openhrp3-source-$DATE/src;
  (echo 'p' | svn co --trust-server-cert --non-interactive https://openrtp.jp/svn/hrg/openhrp/3.1/tags/${OPENHRP3_VERSION} OpenHRP-${OPENHRP3_VERSION}) ||  tar -xzf /tmp/OpenHRP-${OPENHRP3_VERSION}.tgz;
  echo \"* Configure openhrp3 *\";
  mkdir -p ../build;
  cd ../build;
  PKG_CONFIG_PATH=/opt/jsk/lib/pkgconfig:/usr/pkg/lib/pkgconfig:/usr/local/lib/pkgconfig CXX=QCC CC=qcc LDFLAGS=\"-L/usr/local/lib -L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lf2c -latlas -lblas\" cmake ../src/OpenHRP-${OPENHRP3_VERSION} -DEIGEN_INCLUDE_DIR=/opt/jsk/include/eigen3 -DLAPACK_LIBRARY_DIRS=/usr/local/lib -DLAPACK_INCLUDE_DIRS=/usr/local/atlas/include -DOPENRTM_DIR=/opt/jsk -DENABLE_INSTALL_RPATH=ON -DENABLE_INSTALL_RPATH_TO_SELF=ON -DCOMPILE_JAVA_STUFF=OFF -DCMAKE_SHARED_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -latlas -lblas -luuid -lsocket -Wl,-u,MAIN__\" -DCMAKE_EXE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -luuid -lsocket -Wl,-u,MAIN__\" -DCMAKE_MODULE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -latlas -lblas -Wl,-u,MAIN__\" -DCMAKE_INSTALL_PREFIX=/opt/jsk;
  echo \"* Compile openhrp3 *\";
  PATH=/opt/jsk/bin:/usr/pkg/bin:/usr/qnx650/host/qnx6/x86/usr/bin:$PATH LD_LIBRARY_PATH=/opt/jsk/lib:/usr/pkg/lib:/usr/qnx650/host/qnx6/x86/usr/lib make VERBOSE=1;
  echo \"* install openhrp3 *\";
  make install;
  echo \"* make tarball*\";
  cd /tmp;
  tar -czf openhrp3-$OPENHRP3_VERSION-qnx-$DATE.tgz ./openhrp3-source-$DATE/;
  "

hostname=$1
hostname=${hostname:="hiro014"} 
userid=$2
userid=${userid:="hiro"} 
echo "comands = $commands"
read -p "execute compile command @ $hostname (y/n)?"
if [ "$REPLY" == "y" ]; then
    ssh $userid@$hostname -t $commands 2>&1 | tee /tmp/robot-compile-openhrp3-`date +"%Y%m%d-%H%M%S"`.log
    echo "====="
    echo "$ tar -xvzf /tmp/openhrp3-${OPENHRP3_VERSION}-qnx-${DATE}.tgz; cd openhrp3-${OPENHRP3_VERSION}-qnx-$DATE/build; make install"
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

