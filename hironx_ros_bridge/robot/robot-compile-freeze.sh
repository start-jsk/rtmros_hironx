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
f_txt = open('opt_jsk_hex.h','w')
f_txt.write('std::string bin_data="')
f_txt.write(binascii.hexlify(f_exe.read()))
f_txt.write('";')
f_exe.close()
f_txt.close()
EOF

echo -n "input password = (MAC address) "
read PASS
CHECK=`python -c "import hashlib; m=hashlib.md5(); m.update(\"$PASS\"); print m.hexdigest()"`;
echo "check sum ... $CHECK"

cat <<EOF > robot-install.cpp

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <signal.h>

#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <dirent.h>
#include <hw/nicinfo.h> 
#include <sys/dcmd_io-net.h> 

#include "boost/filesystem.hpp"

#include <openssl/md5.h> 

#include "opt_jsk_hex.h"

using namespace std;
using namespace boost::filesystem;

char tmpdir[] = "/tmp/tmp.XXXXXX";
void rm_dir(int signum) {
    remove_all(tmpdir);
    exit(1);
}

int main () {
    // create tmp dir
    mktemp(tmpdir);
    cerr << "tmpdir = " << tmpdir << endl;
    mkdir(tmpdir, S_IRWXU);
    signal(SIGINT, rm_dir);

    // mac address
    char full_int_name[256];
    int fd;
    sprintf(full_int_name, "/dev/io-net/%s","en0"); 
    if ((fd = open(full_int_name, O_RDONLY)) == -1) 
    {
      cerr << "could not open io-net" << endl;
      return(-1); 
    } 
    nic_config_t nicconfig; 
    devctl(fd, DCMD_IO_NET_GET_CONFIG, &nicconfig, sizeof(nicconfig), NULL);
    char buffer[256];
    int i;
    for (i=0; i < 6; i++)
      sprintf(&buffer[i*3], "%02x:", nicconfig.permanent_address[i]);
    buffer[6*3-1] = 0;
    cerr << "mac = " << buffer << endl;

    // md5sum
    unsigned char result[MD5_DIGEST_LENGTH];
    MD5((unsigned char *)buffer, 17, result);
    for(i =0; i < MD5_DIGEST_LENGTH; i++ )
      sprintf(&buffer[i*2], "%02x", result[i]);
    cerr << "md5sum = " << buffer << endl;
    if (string(buffer) != string("$CHECK") ) {
       cerr << "invalid password" << endl;       
       remove_all(tmpdir);
       return -1;
    }

    // create tgz
    string filename = string(tmpdir) + "/out.tgz";
    ofstream fout(filename.c_str(), ios::app);
    for (string::size_type i=0; i<bin_data.length(); i+=2) {
        unsigned char b = (unsigned char) strtoul(bin_data.substr(i,2).c_str(), NULL, 16);
        fout << b;
    }
    fout.close();

    // unzip
    string command = "tar -xvzf "+filename;
    system(command.c_str());

    // remove temp directory
    remove_all(tmpdir);

    return 0;
}

EOF


qcc -o robot-install -I/usr/pkg/include robot-install.cpp -L/usr/pkg/lib -lboost_filesystem -lboost_system -lssl -Wl,-rpath /usr/pkg/lib -Wl,-u,MAIN__
# ./robot-install
cp robot-install ${OUTDIR}/robot-install-${PASS}

rm -fr $TMPDIR


