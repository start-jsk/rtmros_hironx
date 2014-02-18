#!/bin/sh

echo "* Check rtcd"

echo "  Check rtcd log level"

find /opt/jsk/ -name rtcdRobotMode.conf -print -exec grep ^logger.log_level \{} \; 

