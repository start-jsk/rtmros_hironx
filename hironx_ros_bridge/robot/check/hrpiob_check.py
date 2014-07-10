#!/usr/bin/env python

import hrpiob_check_bin
import binascii, tempfile, subprocess, os, shutil, stat

def hrpiob_iob():
    ret = True
    d_tmp = tempfile.mkdtemp()
    bin_name = os.path.join(d_tmp,"hrpiob_check")

    f_bin = open(bin_name,'w')
    f_bin.write(binascii.unhexlify(hrpiob_check_bin.data))
    f_bin.close()
    st = os.stat(bin_name)
    os.chmod(bin_name, st.st_mode | stat.S_IEXEC)

    subprocess.call("ldd %s" % (bin_name), shell=True)
    try:
        if subprocess.check_call(bin_name, shell=True) != 0:
            raise Exception("%s exit with 0"%(bin_name))
    except Exception, e:
        ret = False
        print "  ** libhrpIo.so check failed ...", e.message

    print "  Check libhrpIo.so\t\t\t", ret, "\n"

    shutil.rmtree(d_tmp)  # delete directory
    return ret


