#!/bin/sh

##
## robot-compile-freeze.sh creates an ELF32 binary. It's named as `robot-install-qnx-%MAC_ADDR_QNX%`
## that runs on QNX to install hrpsys (and its other required libraries). 
## The binary doesn't specify the path where the binary files get installed 
## (that is up to users' manual operation in the later process).
## Currently it's assumed to run only on QNX.jenkins.jsk (paths are hardcoded). 
##
## ssh to target QNX
## $ tar -cvzf /tmp/opt_jsk_<hrpsys_version>.tgz /opt/jsk_<hrspys_version>
## run this script 
## input custmer's QNX's MAC address, which is shown in robot-system-check-base
## get robot-install-<MAC address>
##
# set -x

HRPSYS_VERSION_DEFAULT=315.2.8

function usage {
    echo >&2 "usage: $0 [version_hrpsys (default:$HRPSYS_VERSION_DEFAULT)]
    echo >&2 "          [-h|--help] print this message"
    exit 0
}

trap 'exit 1', ERR

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

HRPSYS_VERSION=${3-"$HRPSYS_VERSION_DEFAULT"}
## Path of freeze.py on QNX.jenkins.jsk. This should better be taken from argument in the future. 
FREEZE=/home/sam/hiro-nxo_sys-check/freeze_bin/usr/share/doc/python2.7/examples/Tools/freeze/freeze.py
TMP_FOLDER=`mktemp -d`
OUTDIR=`pwd`

cd $TMP_FOLDER

## Create a "opt_jsk_hex.h" file that encapsulates the streamlined /opt/jsk tarball 
python <<EOF
import binascii
f_exe = open("/tmp/opt_jsk_$HRPSYS_VERSION.tgz", 'r');
f_txt = open('opt_jsk_hex.h', 'w')
f_txt.write('std::string bin_data="')
f_txt.write(binascii.hexlify(f_exe.read()))
f_txt.write('";')
f_exe.close()
f_txt.close()
EOF

## Read MAC address, generate md5 checksum for it.
echo -n "input password = (MAC address) "
read PASS
CHECK=`python -c "import hashlib; m=hashlib.md5(); m.update(\"$PASS\"); print m.hexdigest()"`;
echo "check sum ... $CHECK"

## Write the following text between 2 EOFs into robot-install.cpp
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

#include <sys/socket.h>
#include <net/if_dl.h>
#include <ifaddrs.h>

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
    char buffer[256];

    // create tmp dir
    mktemp(tmpdir);
    cerr << "tmpdir = " << tmpdir << endl;
    mkdir(tmpdir, S_IRWXU);
    signal(SIGINT, rm_dir);

    // mac address
    struct ifaddrs *ifaphead;
    unsigned char * if_mac;
    int found = 0;
    struct ifaddrs *ifap;
    struct sockaddr_dl *sdl = NULL;
    char iface[] = "wm0";

    // TODO: What's this check for? 
    if (getifaddrs(&ifaphead) != 0) {
      perror("get_if_name: getifaddrs() failed");
      exit(1);
    }

    for (ifap = ifaphead; ifap && !found; ifap = ifap->ifa_next) {
      if ((ifap->ifa_addr->sa_family == AF_LINK)) {
        if (strlen(ifap->ifa_name) == strlen(iface))
          if (strcmp(ifap->ifa_name, iface) == 0) {
            found = 1;
            sdl = (struct sockaddr_dl *)ifap->ifa_addr;
            if (sdl) {
              /* I was returning this from a function before converting
               * * this snippet, which is why I make a copy here on the heap */
              if_mac = (unsigned char *)malloc(sdl->sdl_alen);
              memcpy(if_mac, LLADDR(sdl), sdl->sdl_alen);
            }
          }
      }
    }
    if (!found) {
      fprintf (stderr, "Can't find interface %s.\n", iface);
      if(ifaphead)
          freeifaddrs(ifaphead);
      exit(1);
    }

   sprintf (buffer, "%02x:%02x:%02x:%02x:%02x:%02x",
            if_mac[0] , if_mac[1] , if_mac[2] ,
            if_mac[3] , if_mac[4] , if_mac[5] );
   if(ifaphead)
     freeifaddrs(ifaphead);

    // TODO: Indentation is ambiguous from here.
    cerr << "mac = " << buffer << endl;

    // md5sum
    unsigned char result[MD5_DIGEST_LENGTH];
    MD5((unsigned char *)buffer, 17, result);
    int i;
    for(i =0; i < MD5_DIGEST_LENGTH; i++ )
      sprintf(&buffer[i*2], "%02x", result[i]);
    cerr << "md5sum = " << buffer << endl;
    // TODO: Comparing something with MAC address, and return error if not match??? 
    if (string(buffer) != string("$CHECK") ) {
       cerr << "invalid password" << endl;       
       remove_all(tmpdir);
       return -1;
    }

    // create tgz
    string filename = string(tmpdir) + "/out.tgz";
    ofstream fout(filename.c_str(), ios::app);
    for (string::size_type i=0; i<bin_data.length(); i+=2) {
        unsigned char b = (unsigned char) strtoul(bin_data.substr(i, 2).c_str(), NULL, 16);
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

rm -fr $TMP_FOLDER
