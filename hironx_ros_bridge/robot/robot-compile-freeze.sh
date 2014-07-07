#!/bin/sh

##
## ssh to target QNX
## $ tar -cvzf /tmp/opt_jsk.tgz /opt/jsk
## run this script 
## input custmer's QNX's MAC address, which is shown in robot-system-check-base
## get robot-install-<MAC address>
##
# set -x

trap 'exit 1', ERR

FREEZE=/home/sam/hiro-nxo_sys-check/freeze_bin/usr/share/doc/python2.7/examples/Tools/freeze/freeze.py
TMPDIR=`mktemp -d`
OUTDIR=`pwd`

cd $TMPDIR

python <<EOF
import binascii
f_exe = open("/tmp/opt_jsk.tgz", 'r');
f_txt = open('opt_jsk_bin.py','w')
f_txt.write('data="')
f_txt.write(binascii.hexlify(f_exe.read()))
f_txt.write('"')
f_exe.close()
f_txt.close()
EOF

echo -n "input password = (MAC address) "
read PASS
CHECK=`python -c "import hashlib; m=hashlib.md5(); m.update(\"$PASS\"); print m.hexdigest()"`;
echo "check sum ... $CHECK"

cat <<EOF > robot-install.py
#!/usr/bin/env python
import opt_jsk_bin
import binascii, tempfile, subprocess, os, shutil, stat, sys


d_tmp = tempfile.mkdtemp()
bin_name = os.path.join(d_tmp,"out.tgz")

f_bin = open(bin_name,'w')
f_bin.write(binascii.unhexlify(opt_jsk_bin.data))
f_bin.close()
st = os.stat(bin_name)
os.chmod(bin_name, st.st_mode | stat.S_IEXEC)

# check password
import hashlib
m = hashlib.md5()
mac = subprocess.Popen("ifconfig | grep address | sed s/.*address:\ //g", shell=True, stdout=subprocess.PIPE).communicate()[0].rstrip()
m.update("%s"%mac)
print "check if ",m.hexdigest(), " == $CHECK" 
if m.hexdigest() != "$CHECK":
    print "password check is failed, invalid usage"
    sys.exit(1)
# get macaddress
# create pass
# check 
try:
    subprocess.check_call("tar -xvzf %s" % (bin_name), shell=True)
    print "hogeaaa"
except Exception, e:
    print "  ** fail to expand out.tgz ...", e.message

shutil.rmtree(d_tmp)  # delete directory

EOF

python $FREEZE ./robot-install.py
make
cp robot-install ${OUTDIR}/robot-install-${PASS}
# python ./robot-install.py

rm -fr $TMPDIR

