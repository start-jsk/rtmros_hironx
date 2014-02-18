#

echo "* Check OpenHRP3"

cat <<EOF > /tmp/test-openhrp3.cpp
#include <stdio.h>
#include <hrpModel/Config.h>
int main() {
  fprintf(stdout, "%d.%d.%d\n", HRPMODEL_VERSION_MAJOR, HRPMODEL_VERSION_MINOR, HRPMODEL_VERSION_MICRO);
}
EOF

gcc -o /tmp/test-openhrp3 /tmp/test-openhrp3.cpp -I/opt/jsk/include/OpenHRP-3.1 -lstdc++
openhrp3_version=`/tmp/test-openhrp3`

echo -n "  Check OpenHRP3 version\t($openhrp3_version)\t\t\t\t\t";  ([ "$openhrp3_version" == "3.1.5" ] && echo "OK" ) || echo "False"


echo "* Check /opt/hiro/lib/libhrpIo.so"
md5sum=`python -c "import hashlib,os; print hashlib.md5(open('/opt/hiro/lib/libhrpIo.so','rb').read()).hexdigest()"`
size=`python -c "import hashlib,os; print os.path.getsize('/opt/hiro/lib/libhrpIo.so')"`
echo -n "\t   size $size\t\t\t\t\t\t\t";   ([ $size == '1907174' ] && echo "OK" ) || echo "False"
echo -n "\t md5sum $md5sum\t\t\t"; ([ $md5sum == 'e2e07b593dbbf6bbe18328af441af645' ] && echo "OK" ) || echo "False"



echo -n "  Check /opt/hiro/lib/libhrpIo.so is compilable\t\t\t\t"

cat <<EOF > /tmp/test-hrpio.cpp
#include <stdio.h>
#include <sys/types.h>
#include <io/iob.h>
int main() {
  open_iob();
  close_iob();
}
EOF

gcc -o /tmp/test-hrpio /tmp/test-hrpio.cpp -I/opt/jsk/include/hrpsys -L/usr/pkg/lib -L/opt/hiro/lib -lhrpIo -lboost_thread -lboost_signals -lboost_filesystem -lboost_system -lboost_regex -lf2c
ret=$?

if [ $ret == 0 ] ;then
  echo "OK"
else
  echo "False"
  ret=1
fi 


# ldd /opt/hiro/lib/libhrpIo.so  2>&1  | grep ldd

echo -n "  Check ldd of /opt/hiro/lib/libhrpIo.so\t\t\t\t"
LD_LIBRARY_PATH=/usr/pkg/lib:/opt/jsk/lib ldd /opt/hiro/lib/libhrpIo.so  2>&1  | tee | grep ldd
ret=$?

if [ $ret == 1 ] ;then
  echo "OK"
else
  echo "False"
  ret=1
fi 

# ldd /opt/jsk/lib/hrpEC.so  2>&1  | grep ldd

echo "* Check /opt/jsk/lib/hrpEC.so"
md5sum=`python -c "import hashlib,os; print hashlib.md5(open('/opt/jsk/lib/hrpEC.so','rb').read()).hexdigest()"`
size=`python -c "import hashlib,os; print os.path.getsize('/opt/jsk/lib/hrpEC.so')"`
echo -n "\t   size $size\t\t\t\t\t\t\t";   ([ $size == '169895' ] && echo "OK" ) || echo "False"
echo -n "\t md5sum $md5sum\t\t\t"; ([ $md5sum == '17eae811e7ae2a920b445cc2b7c284c8' ] && echo "OK" ) || echo "False"



echo -n "  Check ldd of /opt/jsk/lib/hrpEC.so\t\t\t\t"
LD_LIBRARY_PATH=/usr/pkg/lib:/opt/jsk/lib:/opt/hiro/lib ldd /opt/jsk/lib/hrpEC.so  2>&1  | tee | grep ldd
ret=$?

if [ $ret == 1 ] ;then
  echo "OK"
else
  echo "False"
  ret=1
fi 

