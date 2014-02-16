#!/bin/sh

name=`qconfig -a | grep installation-name | cut -d : -f 2`
base=`qconfig -a | grep installation-base | cut -d : -f 2`
host=`qconfig -a | grep installation-host | cut -d : -f 2`
target=`qconfig -a | grep installation-target | cut -d : -f 2`

ret=0

echo "* Check QNX Info"
echo -n "  Check Installation-name .. $name\t"
if [ "$name" == " QNX Software Development Platform 6.5.0" ]; then
  echo "OK"
else
  echo "False"
  ret=1
fi

echo -n "  Check Installation-base .. $base\t\t\t\t"
if [ "$base" == " /usr/qnx650/" ]; then
  echo "OK"
else
  echo "False"
  ret=1 
fi

                                                                            
echo -n "  Check Installation-host .. $host\t\t"
if [ "$host" == " /usr/qnx650/host/qnx6/x86/" ]; then
  echo "OK"
else
  echo "False"
  ret=1
fi                                                                             
                                                                              
echo -n "  Check Installation-target .. $target\t\t"
if [ "$target" == " /usr/qnx650/target/qnx6/" ]; then
  echo "OK"
else
  echo "False"
  ret=1
fi                                                                             
  
exit $ret
