#!/usr/bin/env python

import sys, tempfile, zipfile, socket, time, subprocess
from qnx_cpu_check import *
from qnx_hdd_check import *
from qnx_eth_check import *
#from qnx_qconfig import *
from hrpiob_check import *
from md5sum_check import *

class Logger(object):
    def __init__(self, filename="Default.log"):
        self.terminal = sys.stdout
        self.log = open(filename, "a")

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
        self.terminal.flush()
        self.log.flush()

hostname = socket.gethostname()
localtime = time.strftime("%Y%m%d-%H%M%S", time.gmtime())
tmp_base_dir = tempfile.mkdtemp()
tmp_dir = tmp_base_dir+'/%s-%s'%(hostname, localtime)
os.mkdir(tmp_dir)
sys.stdout = Logger('%s/robot-system-check-result.log'%(tmp_dir))

try:

    # run cpu check
    print "* Check Environment Variables"
    print "  Check $PATH ... ", os.environ.get('PATH')
    print "  Check $LD_LIBRARY_PATH ... ", os.environ.get('LD_LIBRARY_PATH')
    print "  Check $PYTHON_PATH ... ", os.environ.get('PYTHON_PATH')

    # run cpu check
    print "* Check CPU Info"
    ret = qnx_cpu_check()

    # check hdd space
    print "* Check HDD Info"
    ret = qnx_hdd_check()

    # run qnx config
    #print "* Check QNX Info"
    #ret = qnx_qconfig() and ret

    # check eth
    print "* Check Eth Info"
    ret = qnx_eth_check()

    # check hrpIo.so
    print "* Check libhrpIo.so"
    ret = hrpiob_iob() and ret

    ret = md5sum_check_dir(['/usr/pkg/etc',    0xbeef94e03b158101a77526be67d3b9ecL], output_dir = tmp_dir) and ret # test

    print "* Check /usr/local directories"
    ret = md5sum_check_dir(['/usr/local',      0x1848d9debc509a0f04620f499d14ceb2L], output_dir = tmp_dir) and ret

    print "* Check /usr/pkg directories"
    ret = md5sum_check_dir(['/usr/pkg',         0x91f33b46b798da7d176e55367144903L], output_dir = tmp_dir) and ret
    ret = md5sum_check_dir(['/usr/pkg/bin',    0x3d8eff565b05b7532b3107b20589cbe6L], output_dir = tmp_dir) and ret
    ret = md5sum_check_dir(['/usr/pkg/etc',    0xbeef94e03b158101a77526be67d3b9ecL], output_dir = tmp_dir) and ret
    ret = md5sum_check_dir(['/usr/pkg/include',0xe23c6c662a7acf846d4dd25b63fbe6f4L], output_dir = tmp_dir) and ret
    ret = md5sum_check_dir(['/usr/pkg/info',   0x3e0519bf78f3258cc72f3d095e942ed8L], output_dir = tmp_dir) and ret
    ret = md5sum_check_dir(['/usr/pkg/lib',    0x36d574207bdbb13bc5b898c9870735a6L], output_dir = tmp_dir) and ret
    ret = md5sum_check_dir(['/usr/pkg/man',    0xb97c0f124a4170abd7c170fc882b7425L], output_dir = tmp_dir) and ret
    ret = md5sum_check_dir(['/usr/pkg/sbin',   0x58a3b20eafea4170215f820182f68483L], output_dir = tmp_dir) and ret
    ret = md5sum_check_dir(['/usr/pkg/share',  0x814de4e7c790d11dc4764139b5fb3625L], output_dir = tmp_dir) and ret

    print "* Check /opt/nextage-open directories"
    ret = md5sum_check_dir(['/opt/nextage-open',    0x5644cb667d7241ebec88d79b610b4558L], output_dir = tmp_dir) and ret

    print "* Check /var files"
    import md5sum_var
    ret = md5sum_check_files('/var/db', md5sum_var.info) and ret

    print "* Check /usr/local files"
    import md5sum_usr_local
    ret = md5sum_check_files('/usr/local', md5sum_usr_local.info) and ret

    print "* Check /usr/pkg files"
    import md5sum_usr_pkg_bin
    import md5sum_usr_pkg_etc
    import md5sum_usr_pkg_include
    import md5sum_usr_pkg_info
    import md5sum_usr_pkg_lib
    import md5sum_usr_pkg_man
    import md5sum_usr_pkg_sbin
    import md5sum_usr_pkg_share

    ret = md5sum_check_files('/usr/pkg/bin', md5sum_usr_pkg_bin.info)        and ret
    ret = md5sum_check_files('/usr/pkg/etc', md5sum_usr_pkg_etc.info)        and ret
    ret = md5sum_check_files('/usr/pkg/include', md5sum_usr_pkg_include.info)        and ret
    ret = md5sum_check_files('/usr/pkg/info', md5sum_usr_pkg_info.info)      and ret
    ret = md5sum_check_files('/usr/pkg/lib', md5sum_usr_pkg_lib.info)        and ret
    ret = md5sum_check_files('/usr/pkg/man', md5sum_usr_pkg_man.info)        and ret
    ret = md5sum_check_files('/usr/pkg/sbin', md5sum_usr_pkg_sbin.info)      and ret
    ret = md5sum_check_files('/usr/pkg/share', md5sum_usr_pkg_share.info)    and ret

    print "* Check /opt files"
    import md5sum_opt_nextage_open
    ret = md5sum_check_files('/opt/nextage-open', md5sum_opt_nextage_open.info) and ret

    print ""
    print "---"
    print ""
    print "Done all test, Result is ... ", ret

    if ret:
        print ""
        print "--- !!! CONGRATULATIONS !!! --- "
        print ""

except Exception, e:
    print "*** "
    print "*** Somegthing was wrong...", e.message
    print "*** "
finally:
    passwd = 'tork0808'
    logfilename = 'robot-system-check-result-%s-%s.zip'%(hostname, localtime)
    print ""
    print "---"
    print ""
    print "Saving results.... to %s}"%logfilename

    try:
        subprocess.check_call("cd %s; zip --password %s %s/%s -r ."%(tmp_base_dir, passwd, os.getcwd(), logfilename), shell=True, stdout=subprocess.PIPE)
    except Exception, e:
        print "*** Somegthing was wrong...", e.message
        shutil.rmtree(tmp_base_dir)  # delete directory

    # z = zipfile.ZipFile(logfilename+'.zip','w')
    # for root, dirs, files in os.walk(tmp_dir):
    #     for file in files:
    #         filename = os.path.join(root,file)
    #         print filename, root, dirs, file
    #         z.write(filename, file)
    # z.close()
    shutil.rmtree(tmp_base_dir)  # delete directory

    print "Send following file to maintainers"
    print "  %s"%logfilename
    print ""
    


sys.exit(ret)






