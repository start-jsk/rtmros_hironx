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

wget http://www.openrtm.org/pub/OpenRTM-aist/cxx/1.1.0/OpenRTM-aist-1.1.0-RELEASE.tar.gz -O /tmp/OpenRTM-aist-1.1.0-RELEASE.tar.gz || echo "ERROR:: Failed to download source code"

commands="
  . ~/.profile;
  env;
  trap 'exit 1' ERR;
  set +x;
  echo \"* Download openrtm *\";
  mkdir -p /tmp/openrtm-source-$DATE/src;
  cd /tmp/openrtm-source-$DATE/src;
  mv /tmp/OpenRTM-aist-1.1.0-RELEASE.tar.gz .;
  tar -xzf OpenRTM-aist-1.1.0-RELEASE.tar.gz;
  echo \"* Configure openrtm *\";
  cd OpenRTM-aist-1.1.0;
  sed -i 's/^CPPFLAGS=\"-Wall -fPIC \$CPPFLAGS\"/#CPPFLAGS=\"-Wall -fPIC \$CPPFLAGS\"/g' configure;
  sed -i 's/^CXXFLAGS=\"-Wall -fPIC \$CXXFLAGS\"/#CXXFLAGS=\"-Wall -fPIC \$CXXFLAGS\"/g' configure;
  sed -i 's/^CPPFLAGS=\"-Wall -fPIC \$CPPFLAGS\"/#CPPFLAGS=\"-Wall -fPIC \$CPPFLAGS\"/g' src/lib/coil/configure;
  sed -i 's/^CXXFLAGS=\"-Wall -fPIC \$CXXFLAGS\"/#CXXFLAGS=\"-Wall -fPIC \$CXXFLAGS\"/g' src/lib/coil/configure;
  sed -i '1s/^/#include <stddef.h> /' src/lib/coil/posix/coil/Properties.cpp;
  sed -i '1s/^/#include <ctype.h> /' src/lib/coil/posix/coil/stringutil.cpp;
  sed -i '1s/^/#include <string.h> /' src/lib/coil/posix/coil/Routing.cpp;
  sed -i '1s/^/#include <string.h> /' src/lib/coil/posix/coil/File.h;
  sed -i '1s/^/#include <stdio.h> /' src/lib/rtm/ManagerConfig.cpp;
  sed -i '1s/^/#include <stdio.h> /' src/lib/rtm/ModuleManager.cpp;
  sed -i '1s/^/#include <stdlib.h> /' src/lib/rtm/RTObject.cpp;
  sed -i '1s/^/#include <stdlib.h> /' examples/SimpleIO/ConnectorComp.cpp;
  sed -i '1s/^/#include <stdlib.h> /' examples/ExtTrigger/ConnectorComp.cpp;
  sed -i 's/COIL_OS_LINUX/COIL_OS_QNX/' src/lib/coil/posix/coil/Routing.cpp;
  sed -i 's/COIL_OS_LINUX/COIL_OS_QNX/' src/lib/coil/posix/coil/UUID.cpp;
  sed -i 's/COIL_OS_LINUX/COIL_OS_QNX/' src/lib/coil/posix/coil/UUID.h;
  PATH=/usr/pkg/bin:/usr/qnx650/host/qnx6/x86/usr/bin:$PATH PKG_CONFIG_PATH=/usr/pkg/lib/pkgconfig CXX=QCC CC=qcc LDFLAGS=\"-L/opt/jsk/lib -L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -luuid -lsocket -Wl,-u,MAIN__\" CPPFLAGS=\"-I/opt/jsk/include -I/usr/pkg/include -Wall -O2 -I../../include/ \" CXXFLAGS=\"-I/opt/jsk/include -I/usr/pkg/include -Wall -O2 -I../../include/ \" ./configure --host x86-linux-gnu --prefix=/opt/jsk --with-pic=no;

  echo \"* Compile openrtm *\";
  PATH=/usr/pkg/bin:/usr/qnx650/host/qnx6/x86/usr/bin:$PATH LD_LIBRARY_PATH=/usr/pkg/lib:/usr/qnx650/host/qnx6/x86/usr/lib make VERBOSE=1;

  echo \"* install openrtm *\";
  PATH=/usr/pkg/bin:/usr/qnx650/host/qnx6/x86/usr/bin:$PATH LD_LIBRARY_PATH=/usr/pkg/lib:/usr/qnx650/host/qnx6/x86/usr/lib make install;

  echo \"* make tarball*\";
  cd /tmp;
  tar -czf openrtm-qnx-$DATE.tgz ./openrtm-source-$DATE;
  "

hostname=$1
hostname=${hostname:="hiro014"} 
userid=$2
userid=${userid:="hiro"} 
echo "comands = $commands"
read -p "execute compile command @ $hostname (y/n)? "
if [ "$REPLY" == "y" ]; then
    scp /tmp/OpenRTM-aist-1.1.0-RELEASE.tar.gz $userid@$hostname:/tmp/
    ssh $userid@$hostname -t $commands 2>&1 | tee /tmp/robot-compile-openrtm-`date +"%Y%m%d-%H%M%S"`.log
    echo "====="
    echo "Log on to $userid@$hostname, then run the following:"
    echo "$ tar -xvzf /tmp/openrtm-qnx-$DATE.tgz; cd openrtm-source-$DATE/src/OpenRTM-aist-1.1.0/build; make install"
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

