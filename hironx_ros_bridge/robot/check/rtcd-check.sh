#!/bin/sh

echo "* Check rtcd"

echo "  Check rtcd log level"

for file in `find /opt/jsk -name rtcdRobotMode.conf`; do
    log_level=`grep ^logger.log_level $file | sed s@\\\s@@g`
    echo -n "   $file ($log_level)\t"
    RET=`echo ${log_level} | grep -c 'logger.log_level:NORMAL'`
    if [ $RET -eq 1 ]; then
	echo "OK";
    else
	echo "Flase";
    fi
done



