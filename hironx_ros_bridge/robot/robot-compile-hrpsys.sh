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

address=`host hrpsys-base.googlecode.com | awk '/^[[:alnum:].-]+ has address/ { print $4 ; exit }'` # this does not work for  Server certificate verification 

commands="
  . ~/.profile;
  echo \"* Download hrpsys *\";
  mkdir -p src;
  cd src;
  svn co http://hrpsys-base.googlecode.com/svn/trunk hrpsys-base-source;
  echo \"* Configure hrpsys *\";
  cd hrpsys-base-source;
  PKG_CONFIG_PATH=/opt/jsk/lib/pkgconfig:/usr/pkg/lib/pkgconfig CXX=QCC CC=qcc TVMET_DIR=/opt/jsk OPENRTM_IDL_DIR=/opt/jsk/iclude/OpenRTM-1.1/rtm/idl LDFLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem\"  cmake . -DLAPACK_LIBRARY_DIRS=/opt/jsk/lib -DLAPACK_INCLUDE_DIRS=/opt/jsk/include -DOPENRTM_DIR=/opt/jsk -DOPT_COLLADASUPPORT=NO -DEIGEN_INCLUDE_DIR=/opt/jsk/inclde/eigen3 -DCOMPILE_JAVA_STUFF=OFF -DCMAKE_SHARED_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_EXE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_MODULE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex lf2c -Wl,-u,MAIN__\" -DCMAKE_INSTALL_PREFIX=/opt/jsk -DENABLE_DOXYGEN=OFF ;
  echo \"* Compile hrpsys *\";
  make;
  echo \"* Install hrpsys *\";
  su -c 'make install; ln -sf /opt/hiro/lib/libhrpIo.so /opt/jsk/lib/libhrpIo.so';
  ls -al /opt/jsk/lib/libhrpIo.so;
  cd ec/hrpEC;
  touch  hrpEC.cpp;
  make VERBOSE=1;
  /usr/qnx650/host/qnx6/x86/usr/bin/QCC   -O3 -DNDEBUG -L/usr/pkg/lib /usr/pkg/lib/libboost_thread.a -lboost_thread -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -Wl,-u,MAIN__ -shared -Wl,-soname,hrpEC.so -o ../../lib/hrpEC.so CMakeFiles/hrpEC.dir/hrpEC.o CMakeFiles/hrpEC.dir/hrpEC-common.o -L/opt/jsk/lib -L/usr/pkg/lib -Lio -luuid -lsocket -lm -lomniORB4 -lomnithread -lomniDynamic4 -lRTC -lcoil  /opt/jsk/lib/libhrpIo.so /opt/jsk/lib/libhrpsysBaseStub.so -luuid -lsocket -lm -lomniORB4 -lomnithread -lomniDynamic4 -lRTC -lcoil -Wl,-rpath,/opt/jsk/lib:/usr/pkg/lib:io;
  su -c 'cp ../../lib/hrpEC.so /opt/jsk/lib/';
  ldd /opt/jsk/lib/hrpEC.so;
  "

hostname=$1
hostname=${hostname:="hiro014"} 
read -p "execute compile command @ $hostname (y/n)?"
if [ "$REPLY" == "y" ]; then
    ssh hiro@$hostname -t $commands
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

