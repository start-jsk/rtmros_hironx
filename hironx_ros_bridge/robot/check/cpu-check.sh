#!/bin/sh

cpu=`pidin info | head -1 | cut -d\   -f 1`
memory=`pidin info | head -1 | cut -d\/   -f 2 | cut -d\  -f 1`
release=`pidin info | head -1 | cut -d\   -f 2`
cpunum=`pidin info | tail -1 | cut -d\   -f 1`

ret=0

echo "* Check CPU Info"
echo -n "  Check CPU Type .. $cpu\t\t\t\t\t\t"
if [ "$cpu" == "CPU:X86" ]; then
  echo "OK"
else
  echo "False"
  ret=1
fi

echo -n "  Check Number of CPU .. $cpunum\t\t\t\t\t"
if [ "$cpunum" == "Processor2:" ]; then
  echo "OK"
else
  echo "False"
  ret=1
fi

echo -n "  Check Memory Size .. $memory\t\t\t\t\t\t"
if [ "$memory" == "3318Mb" ]; then
  echo "OK"
else
  echo "False"
  ret=1 
fi
                           
echo -n "  Check OS Release .. $release\t\t\t\t\t"
if [ "$release" == "Release:6.5.0" ]; then
  echo "OK"
else
  echo "False"
  ret=1
fi                                                                             
  
exit $ret
