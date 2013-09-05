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

commands="
  echo \"* Restart NameServer *\";
  nohup /opt/jsk/bin/NameServer.sh > /dev/null;
  echo \"* Restart ModelLoader *\";
  nohup /opt/jsk/bin/ModelLoader.sh > /dev/null;
  echo \"* Sleep 3sec *\";
  /usr/bin/env sleep 3;
  echo \"* Restart rtcd *\";
  slay -9 rtcd;
  echo \"* Finish Restart NameServer, ModelLoader and rtcd *\";
  "

hostname=$1

echo "execute restart command @" ${hostname:="hiro014"}
ssh root@${hostname:="hiro014"} -t $commands
