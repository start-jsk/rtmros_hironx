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


# ldd /opt/jsk/lib/hrpEC.so  2>&1  | grep ldd

echo "* Check /opt/jsk/lib/hrpEC.so"
md5sum=`python -c "import hashlib,os; print hashlib.md5(open('/opt/jsk/lib/hrpEC.so','rb').read()).hexdigest()"`
size=`python -c "import hashlib,os; print os.path.getsize('/opt/jsk/lib/hrpEC.so')"`
echo -n "\t   size $size\t\t\t\t\t\t\t";   ([ $size == '169935' ] && echo "OK" ) || echo "False"
echo -n "\t md5sum $md5sum\t\t\t"; ([ $md5sum == 'cc859cf9e12f467e1e0c007054562ed1' ] && echo "OK" ) || echo "False"



echo -n "  Check ldd of /opt/jsk/lib/hrpEC.so\t\t\t\t"
LD_LIBRARY_PATH=/usr/pkg/lib:/opt/jsk/lib:/opt/hiro/lib ldd /opt/jsk/lib/hrpEC.so  2>&1  | tee | grep ldd
ret=$?

if [ $ret == 1 ] ;then
  echo "OK"
else
  echo "False"
  ret=1
fi 

