#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro014)]"
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

PID=28169

wget https://raw.github.com/start-jsk/openhrp3/master/package.xml -O /tmp/openhrp3-package.xml
OPENHRP3_VERSION=`sed -n 's@\s*<version>\(.*\)</version>@\1@p' /tmp/openhrp3-package.xml`


commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;
  echo \"* Download openhrp3 *\";
  mkdir -p /tmp/openhrp3-source-$PID/src;
  cd /tmp/openhrp3-source-$PID/src;
  (echo 'p' | svn co https://openrtp.jp/svn/hrg/openhrp/3.1/tags/${OPENHRP3_VERSION} OpenHRP-${OPENHRP3_VERSION}) ||  tar -xzf /tmp/OpenHRP-${OPENHRP3_VERSION}.tgz;
  echo \"* Configure openhrp3 *\";
  cd OpenHRP-${OPENHRP3_VERSION};
  PKG_CONFIG_PATH=/opt/jsk/lib/pkgconfig:/usr/pkg/lib/pkgconfig CXX=QCC CC=qcc LDFLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem\" cmake . -DEIGEN_INCLUDE_DIR=/opt/jsk/include/eigen3 -DLAPACK_LIBRARY_DIRS=/opt/jsk/lib -DLAPACK_INCLUDE_DIRS=/opt/jsk/include -DOPENRTM_DIR=/opt/jsk -DENABLE_INSTALL_RPATH=ON -DENABLE_INSTALL_RPATH_TO_SELF=ON -DCOMPILE_JAVA_STUFF=OFF -DCMAKE_SHARED_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_EXE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_MODULE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex lf2c -Wl,-u,MAIN__\" -DCMAKE_INSTALL_PREFIX=../../;
  echo \"* Compile openhrp3 *\";
  PATH=/opt/jsk/bin:/usr/pkg/bin:/usr/qnx650/host/qnx6/x86/usr/bin:$PATH LD_LIBRARY_PATH=/opt/jsk/lib:/usr/pkg/lib:/usr/qnx650/host/qnx6/x86/usr/lib make VERBOSE=1;
  make install;
  echo \"* Modify codes *\";
  cd /tmp;
  (cd openhrp3-source-$PID; sed -i s@/tmp/openhrp3-source-$PID@/opt/jsk@g lib/pkgconfig/openhrp3.1.pc bin/openhrp-jython-prompt include/OpenHRP-3.1/hrpModel/Config.h;);
  echo \"* make tarball*\";
  tar -C openhrp3-source-$PID  -czf openhrp3-qnx-${OPENHRP3_VERSION}.tgz ./;
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
    echo "$ cd /opt/jsk"
    echo "$ tar -xvzf /tmp/openhrp3-qnx-${OPENHRP3_VERSION}.tgz"
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

