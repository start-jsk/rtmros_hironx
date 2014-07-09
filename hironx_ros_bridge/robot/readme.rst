
description
========================
This file explains the steps of installing opensource contoller on QNX for Hiro - Nextage Open robot.

Requirement
========================
- A user account on the robot's QNX, write-access to `/opt/jsk`.

Installation steps
========================
Assume you're at `hironx_ros_bridge/robot`.

1. Run remote check script (need better name) against runtime QNX host.
::

  $ robot-system-check.sh %QNX_HOSt_NAME% %QNX_USER%

- Expected result example (Tobe updated. No success example available)

::

  $ ./robot-system-check.sh qnx-runtime tork
  ;; check Ubuntu version precise
  PING qnx-runtime (qnx-runtime) 56(84) bytes of data.
  64 bytes from qnx-runtime: icmp_req=1 ttl=255 time=0.391 ms
  
  --- qnx-runtime ping statistics ---
  1 packets transmitted, 1 received, 0% packet loss, time 0ms
  rtt min/avg/max/mdev = 0.391/0.391/0.391/0.000 ms
  Host 10.128.168.192.in-addr.arpa. not found: 3(NXDOMAIN)
  -- [ERROR] Could find IP address/Host name for qnx-runtime
  hit any key to exit

- Files that get installed: None.

2. Send check scripts (need better name) to QNX. 
::

  $ tar cfvz robot-check.tgz ./check/*
  $ scp robot-check.tgz tork@qnx-dev:/home/tork

- Expected result example (Tobe updated. No success example available)

3. Log on to QNX with developer license, build check scripts.
::

  $ ssh tork@qnx-dev
  qnx-dev$ tar xfvz robot-check.tgz -d robot-check && cd robot-check
  qnx-dev$ make
  qnx-dev$ ls
           robot-system-check-base-2014-07-09
  qnx-dev$ logout
  $ scp tork@qnx-dev:/home/tork/robot-system-check-base-2014-07-09 .

4. Run the generated checker binary on runtime QNX (w/o develoer license)
:: 

  $ scp robot-system-check-base-2014-07-09 tork@qnx-runtime:/home/tork/
  qnx-runtime$ ./robot-system-check-base-2014-07-09

- Expected result example (Needs tobe updated. No success example available)

::

  * Check CPU Info
    Check CPU Type ..  CPU:X86                        Ok
    Check Memory Size ..  FreeMem:2737Mb/3318Mb                       False
    Check OS Release ..  Release:6.5.0                        Ok
    Check Number of CPU  ..  Processor2:                      Ok
  
    -CPU:X86 Release:6.5.0  FreeMem:2737Mb/3318Mb BootTime:Jul 03 14:31:31 UTC 2014
    -Processes: 37, Threads: 111
    -Processor1: 66222 Pentium III Stepping 10 2801MHz FPU 
    -Processor2: 66222 Pentium III Stepping 10 2801MHz FPU 
    -
  * Check HDD Info
    Check HDD Space ..    Total 14401 MB, Used 7072 MB, Free 7329 MB          True
  * Check libhrpIo.so
    ** libhrpIo.so check failed ... /tmp/tmpzgm0vK/hrpiob_check exit with 0
    Check libhrpIo.so         False 
  
    Check  /usr/pkg/etc       ( 0xbeef94e03b158101a77526be67d3b9ecL ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg_etc.py
  * Check /usr/local directories
    Check  /usr/local     ( 0x1848d9debc509a0f04620f499d14ceb2L ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_local.py
  * Check /usr/pkg directories
    Check  /usr/pkg       ( 0x91f33b46b798da7d176e55367144903L )  Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg.py
    Check  /usr/pkg/bin       ( 0x3d8eff565b05b7532b3107b20589cbe6L ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg_bin.py
    Check  /usr/pkg/etc       ( 0xbeef94e03b158101a77526be67d3b9ecL ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg_etc.py
    Check  /usr/pkg/include       ( 0xe23c6c662a7acf846d4dd25b63fbe6f4L ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg_include.py
    Check  /usr/pkg/info      ( 0x3e0519bf78f3258cc72f3d095e942ed8L ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg_info.py
    Check  /usr/pkg/lib       ( 0x36d574207bdbb13bc5b898c9870735a6L ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg_lib.py
    Check  /usr/pkg/man       ( 0xb97c0f124a4170abd7c170fc882b7425L ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg_man.py
    Check  /usr/pkg/sbin      ( 0x58a3b20eafea4170215f820182f68483L ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg_sbin.py
    Check  /usr/pkg/share     ( 0x814de4e7c790d11dc4764139b5fb3625L ) Ok
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_usr_pkg_share.py
  * Check /opt/nextage-open directories
    Check  /opt/nextage-open      ( 0xeae8e97d79396b27df5b558e5aad131eL ) False
    Writingg results to ...  /tmp/tmpR5rqDk/nxc100-20140703-144921/md5sum_opt_nextage_open.py
  * Check /var files
  * Check /usr/local files
  * Check /usr/pkg files
  * Check /opt files
   ** /opt/nextage-open/include/nextage-open.hpp       is possibly newly added (not found on database).
   ** /opt/nextage-open/bin/unlock_iob         is possibly newly added (not found on database).
   ** /opt/nextage-open/bin/md5list.txt        is possibly newly added (not found on database).
   ** /opt/nextage-open/bin/diagnosisServer        is possibly newly added (not found on database).
   ** /opt/nextage-open/bin/NxOpenCore         is possibly newly added (not found on database).
   ** /opt/nextage-open/bin/startNextageOpen.sh        is possibly newly added (not found on database).
  
  ---
  
  Done all test, Result is ...  False
  
  ---
  
  Saving results.... to robot-system-check-result-nxc100-20140703-144921.zip}

5. Run copy-paste script against runtime QNX host.
::

  $ robot-compile-freeze.sh qnx-runtime tork
  :
  (prompt) input password = (MAC address) 
  :
  (Tobe filled.)


- Expected result example (Tobe filled)
