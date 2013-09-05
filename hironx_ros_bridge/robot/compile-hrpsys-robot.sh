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

address=`host hrpsys-base.googlecode.com | awk '/^[[:alnum:].-]+ has address/ { print $4 ; exit }'`

commands="
  echo \"* Download hrpsys *\";
  mkdir -p src2
  cd src2
  svn co https://${address}/svn/trunk hrpsys-base-source
  echo \"* Configure hrpsys *\";
  cd hrpsys-base-source;
  PKG_CONFIG_PATH=/opt/jsk/lib/pkgconfig:/usr/pkg/lib/pkgconfig CXX=QCC CC=qcc TVMET_DIR=/opt/jsk OPENRTM_IDL_DIR=/opt/jsk/iclude/OpenRTM-1.1/rtm/idl LDFLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem\"  cmake . -DLAPACK_LIBRARY_DIRS=/opt/jsk/lib -DLAPACK_INCLUDE_DIRS=/opt/jsk/include -DOPENRTM_DIR=/opt/jsk -DOPT_COLLADASUPPORT=NO -DEIGEN_INCLUDE_DIR=/opt/jsk/inclde/eigen3 -DCOMPILE_JAVA_STUFF=OFF -DCMAKE_SHARED_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_EXE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_MODULE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_regex lf2c -Wl,-u,MAIN__\" -DCMAKE_INSTALL_PREFIX=/opt/jsk -DENABLE_DOXYGEN=OFF 
  echo \"* Compile hrpsys *\";
  PATH=/opt/jsk/bin:$PATH make
  echo \"* Install hrpsys *\";
  PATH=/opt/jsk/bin:$PATH su -c 'make install'
  "
hostname=$1
hostname=${hostname:="hiro014"} 
read -p "execute restart command @ $hostname (y/n)?"
if [ "$REPLY" == "y" ]; then
    ssh root@$hostname -t $commands
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

