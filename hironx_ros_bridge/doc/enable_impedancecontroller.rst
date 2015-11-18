***************************************************************************
Enable Impedance Control with hrpsys particularly with HiroNXO robots
***************************************************************************

Prerequisite
==========================

 * Access to QNX machine with developer license (where you can build by `g++`)
 * Your QNX needs an internet access.
 * Have source of hrpsys (in this doc it's at /opt/jsk/src/hrpsys-base-source-315.1.7. Newer version should work)

Steps
==========================

1. Prepare QNX driver. 
------------------------

1.1. Send `nitta_forcesensor.tgz` to your QNX. Unzip it.::
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   Ubuntu$ roscd hironx_ros_bridge/robot && tar cfvz nitta_forcesensor.tgz nitta | scp qnxuser@yourqnx:/home/qnxuser
   yourqnx$ mv nitta_forcesensor.tgz nitta_dir && cd nitta_dir && tar xfvz nitta_forcesensor.tgz

1.2 Run make.
~~~~~~~~~~~~~~~~~~~~~~~~

 QNX$ cd nitta && make

This might fail depending on the internet connectivity of your QNX. If this fails, go to 1.2.b. Also, beware of https://github.com/start-jsk/rtmros_hironx/issues/271#issuecomment-58293363. Result would look like this::

    $ make all
    g++ -o jr3_driver jr3_driver.cpp
    $ ls
    Makefile                              jr3_driver.cpp
    QNX101                                sample
    jr3_driver                            sample.c

1.2.b. (only when 1.2 failed) Obtain proprietary driver software
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Done by the following command (same as done in `Makefile` (https://github.com/start-jsk/rtmros_hironx/blob/3d355520f01b4e97c09072e1abe12f72baf69568/hironx_ros_bridge/robot/nitta/Makefile#L10)::

    Ubuntu$ wget http://www.nitta.co.jp/images/product/sensor/pdf_ifs/qnx101.zip -O qnx.zip

1.3. Place the generated file `jr3_driver` into `/opt/jsk/bin`.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1.4. Create log file for the driver.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    touch /opt/jsk/var/log/jr3.log

2. Next up is to edit RobotHardware in `hrpsys`.
----------------------------------------------------------------

2.1. Edit robot.cpp
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    $ zile /opt/jsk/src/hrpsys-base-source-315.1.7/rtc/RobotHardware/robot.cpp

diff on our case is as following.::

    --- /opt/jsk/src/hrpsys-base-source-315.1.7/rtc/RobotHardware/robot.cpp.org	2014-02-19 07:25:46.000000000 +0000
    +++ /opt/jsk/src/hrpsys-base-source-315.1.7/rtc/RobotHardware/robot.cpp	2014-09-14 23:29:30.000000000 +0000
    @@ -18,6 +18,85 @@
     
     using namespace hrp;
     
    +//BEGIN: For Force Sensor. See https://github.com/start-jsk/rtmros_hironx/pull/253#issuecomment-57050332
    +#include <stdio.h>
    +#include <unistd.h>
    +#include <stdlib.h>
    +#include <fcntl.h>
    +#include <errno.h>
    +#include <string.h>
    +#include <sys/neutrino.h>
    +#include <sys/iofunc.h>
    +#include <sys/dispatch.h>
    +#include <sys/mman.h>
    +
    +typedef struct
    +{
    +   uint16_t msg_no;
    +   char msg_data[255];
    +} client_msg_t;
    +static int force_sensor_fd;
    +static float force_data[12];
    +
    +int _number_of_force_sensors()
    +{
    +   int ret, num;
    +   client_msg_t msg;
    +   char msg_reply[255];
    +
    +   /* Open a connection to the server (fd == coid) */
    +   force_sensor_fd = open ("/dev/jr3q", O_RDWR);
    +   if (force_sensor_fd == -1)
    +     {
    + fprintf (stderr, "Unable to open server connection: %s\n",
    + strerror (errno));
    + return EXIT_FAILURE;
    +     }
    +
    +
    +   /* Clear the memory for the msg and the reply */
    +   memset (&msg, 0, sizeof (msg));
    +   memset (&msg_reply, 0, sizeof (msg_reply));
    +
    +   /* Setup the message data to send to the server */
    +   num = 4;
    +   msg.msg_no = _IO_MAX + num;
    +   snprintf (msg.msg_data, 254, "client %d requesting reply.", getpid ());
    +
    +   printf ("client: msg_no: _IO_MAX + %d\n", num);
    +   fflush (stdout);
    +   ret = MsgSend (force_sensor_fd, &msg, sizeof (msg), msg_reply, 255);
    +   if (ret == -1)
    +     {
    + fprintf (stderr, "Unable to MsgSend() to server: %s\n",
    + strerror (errno));
    + return EXIT_FAILURE;
    +     }
    +   printf ("client: msg_reply:\n%s\n", msg_reply);
    +
    +   return 2;
    +}
    +int _set_number_of_force_sensors(int num)
    +{
    +   return 2;
    +}
    +int _read_force_sensor(int id, double *forces)
    +{
    +   for(int i=0;i < 6; i++){
    +      forces[i] = force_data[6*id+i];
    +  }
    +   return 0;
    +}
    +int _read_force_offset(int id, double *offsets)
    +{
    +   return 0;
    +}
    +
    +int _write_force_offset(int id, double *offsets)
    +{
    +   return 0;
    +}
    +//END: For Force Sensor
     
     robot::robot() : m_fzLimitRatio(0), m_maxZmpError(DEFAULT_MAX_ZMP_ERROR), m_calibRequested(false), m_pdgainsFilename("PDgains.sav"), wait_sem(0)
     {
    @@ -64,7 +143,7 @@
     
     
         set_number_of_joints(numJoints());
    -    set_number_of_force_sensors(numSensors(Sensor::FORCE));
    +    _set_number_of_force_sensors(numSensors(Sensor::FORCE));
         set_number_of_gyro_sensors(numSensors(Sensor::RATE_GYRO));
         set_number_of_accelerometers(numSensors(Sensor::ACCELERATION));
     
    @@ -72,12 +151,12 @@
         accel_sum.resize(numSensors(Sensor::ACCELERATION));
     
         if ((number_of_joints() != numJoints())
    -	|| (number_of_force_sensors() != numSensors(Sensor::FORCE))
    +	|| (_number_of_force_sensors() != numSensors(Sensor::FORCE))
     	|| (number_of_gyro_sensors() != numSensors(Sensor::RATE_GYRO))
     	|| (number_of_accelerometers() != numSensors(Sensor::ACCELERATION))){
           std::cerr << "VRML and IOB are inconsistent" << std::endl;
           std::cerr << "  joints:" << numJoints() << "(VRML), " << number_of_joints() << "(IOB)"  << std::endl;
    -      std::cerr << "  force sensor:" << numSensors(Sensor::FORCE) << "(VRML), " << number_of_force_sensors() << "(IOB)"  << std::endl;
    +      std::cerr << "  force sensor:" << numSensors(Sensor::FORCE) << "(VRML), " << _number_of_force_sensors() << "(IOB)"  << std::endl;
           std::cerr << "  gyro sensor:" << numSensors(Sensor::RATE_GYRO) << "(VRML), " << number_of_gyro_sensors() << "(IOB)"  << std::endl;
           std::cerr << "  accelerometer:" << numSensors(Sensor::ACCELERATION) << "(VRML), " << number_of_accelerometers() << "(IOB)"  << std::endl;
           return false;
    @@ -93,9 +172,9 @@
         double force[6], offsets[6];
         
         for (int i=0; i<numSensors(Sensor::FORCE); i++) {
    -        read_force_sensor(i, force);
    +        _read_force_sensor(i, force);
             for (int j=0; j<6; j++) offsets[j] = -force[j];
    -        write_force_offset(i, offsets);
    +        _write_force_offset(i, offsets);
         }
     }
     
    @@ -240,6 +319,22 @@
     void robot::oneStep()
     {
         calibrateInertiaSensorOneStep();
    +    //BEGIN: For force sensor. see https://github.com/start-jsk/rtmros_hironx/pull/253#issuecomment-57050332
    +    int num = 1, ret;
    +    char msg_reply[255];
    +    client_msg_t msg;
    +    msg.msg_no = _IO_MAX + num;
    +    ret = MsgSend (force_sensor_fd, &msg, sizeof (msg), msg_reply, 255);
    +    if (ret == -1)
    +      {
    +        fprintf (stderr, "Unable to MsgSend() to server: %s\n",
    +        strerror (errno));
    +        return ;
    +      }
    +    memcpy (force_data, msg_reply, sizeof(float)*12);
    +
    +    calibrateInertiaSensorOneStep();
    +    //END: For force sensor. see https://github.com/start-jsk/rtmros_hironx/pull/253#issuecomment-57050332
         gain_control();
         if (m_calibRequested){
             ::initializeJointAngle(m_calibJointName.c_str(), 
    @@ -398,7 +493,7 @@
     
     void robot::readForceSensor(unsigned int i_rank, double *o_forces)
     {
    -    read_force_sensor(i_rank, o_forces);
    +    _read_force_sensor(i_rank, o_forces);
     }
     
     void robot::writeJointCommands(const double *i_commands)
    @@ -495,7 +590,7 @@
     
         if (m_rLegForceSensorId >= 0){
             double force[6];
    -        read_force_sensor(m_rLegForceSensorId, force);
    +        _read_force_sensor(m_rLegForceSensorId, force);
             if (force[FZ] > totalMass()*G*m_fzLimitRatio){
     	    std::cerr << time_string() << ": right Fz limit over: Fz = " << force[FZ] << std::endl;
                 o_reason = EMG_FZ;
    @@ -505,7 +600,7 @@
         } 
         if (m_lLegForceSensorId >= 0){
             double force[6];
    -        read_force_sensor(m_lLegForceSensorId, force);
    +        _read_force_sensor(m_lLegForceSensorId, force);
             if (force[FZ] > totalMass()*G*m_fzLimitRatio){
     	    std::cerr << time_string() << ": left Fz limit over: Fz = " << force[FZ] << std::endl;
                 o_reason = EMG_FZ;
    
https://github.com/start-jsk/rtmros_hironx/pull/253#issuecomment-57050332

2.2. Compile hrpsys.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

2.2.1. 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    $ wget https://github.com/tork-a/hrpsys-base/archive/315.1.10_add_forcesensor.zip

2.2.1.b. (NOT recommended) Alternatively, replace source with the change made in 2.1.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    qnx$ cd /tmp/hrpsys-source-315.1.10-2014-10-16/src/hrpsys-base-315.1.10/rtc/RobotHardware
    qnx$ cp robot.cpp robot.cpp.org
    qnx$ (Replace the modified file)
    qnx$ cd /tmp/hrpsys-source-315.1.10-2014-10-16/build  (Create if the dir `build` doesn'nt exist)

2.3 (Only if you've done the above compilation on the QNX away from the working robot) Create an archive of the built files.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Then Move it to the working robot.

    qnx-dev$ cd /home/qnxuser && tar cfvz hrpsys315.1.7_with_forcesensor_opt_jsk.tgz /opt/jsk
    (Move hrpsys315.1.7_with_forcesensor_opt_jsk.tgz to the working robot)

3. On the robot QNX, add the modification.
---------------------------------------------

3.1. (Only if you've done 2.3) Melt the archive file.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    qnx-robot$ tar xfvz hrpsys315.1.7_with_forcesensor_opt_jsk.tgz

3.2. Copy the following files into designated location:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 * `RobotHardware.so` at `/opt/jsk/lib/RobotHardware.so`
 * `j3_driver` under `/opt/jsk/bin`

3.2.b (not recommended) Stop the running processes in order to be able to replace lib files. Processes you might want to stop are:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 * rtcd
 * /opt/jsk/bin/omniNames
 * openhrp-model-loader
 * ./shm_iob_hiro (TODO: not sure if this is right)

 To stop::
 
    qnx-robot$ ps -ef | grep %NAME_PROCESSES%
    qnx-robot$ su -c 'kill -9 %PROCESS_ID%'

3.3. Edit `VRML` model file `main.wrl` to add force sensor node. 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Ref. https://github.com/start-jsk/rtmros_hironx/pull/253#issuecomment-57050332

See an example diff at Appendix-A (it's just an example because the model might differ depending on your customization). 

3.4. Modify the start script of the robot `/opt/jsk/bin/startJSK.sh`. 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the following at the bottom.

::

    ./jr3_driver > /opt/jsk/var/log/jr3.log &

Now the file should look like this::

    #!/bin/sh
    
    DATE=`date +%y%m%d_%H%M%S`
    export LD_LIBRARY_PATH=/opt/hiro/bin
    slay shm_iob_hiro
    slay hand_iob
    cd /opt/hiro/bin
    ./shared init
    ./shm_iob_hiro > /opt/hiro/log/shm_iob_hiro_$DATE.log 2>&1 &
    
    cd /opt/jsk/bin
    ./jr3_driver > /opt/jsk/var/log/jr3.log 2>&1 & # For nitta force sensor
    ./NameServer.sh
    ./ModelLoader.sh
    /usr/bin/env sleep 3
    ./rtcd.sh &

3.4. Reboot QNX. Ref. http://wiki.ros.org/rtmros_nextage/Tutorials/Operation%20on%20QNX
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Appendix-A.::

    --- /opt/jsk/src/hrpsys-base-source-315.1.7/rtc/RobotHardware/robot.cpp.org	2014-02-19 07:25:46.000000000 +0000
    +++ /opt/jsk/src/hrpsys-base-source-315.1.7/rtc/RobotHardware/robot.cpp	2014-09-14 23:29:30.000000000 +0000
    @@ -18,6 +18,85 @@
     
     using namespace hrp;
     
    +//BEGIN: For Force Sensor. See https://github.com/start-jsk/rtmros_hironx/pull/253#issuecomment-57050332
    +#include <stdio.h>
    +#include <unistd.h>
    +#include <stdlib.h>
    +#include <fcntl.h>
    +#include <errno.h>
    +#include <string.h>
    +#include <sys/neutrino.h>
    +#include <sys/iofunc.h>
    +#include <sys/dispatch.h>
    +#include <sys/mman.h>
    +
    +typedef struct
    +{
    +   uint16_t msg_no;
    +   char msg_data[255];
    +} client_msg_t;
    +static int force_sensor_fd;
    +static float force_data[12];
    +
    +int _number_of_force_sensors()
    +{
    +   int ret, num;
    +   client_msg_t msg;
    +   char msg_reply[255];
    +
    +   /* Open a connection to the server (fd == coid) */
    +   force_sensor_fd = open ("/dev/jr3q", O_RDWR);
    +   if (force_sensor_fd == -1)
    +     {
    + fprintf (stderr, "Unable to open server connection: %s\n",
    + strerror (errno));
    + return EXIT_FAILURE;
    +     }
    +
    +
    +   /* Clear the memory for the msg and the reply */
    +   memset (&msg, 0, sizeof (msg));
    +   memset (&msg_reply, 0, sizeof (msg_reply));
    +
    +   /* Setup the message data to send to the server */
    +   num = 4;
    +   msg.msg_no = _IO_MAX + num;
    +   snprintf (msg.msg_data, 254, "client %d requesting reply.", getpid ());
    +
    +   printf ("client: msg_no: _IO_MAX + %d\n", num);
    +   fflush (stdout);
    +   ret = MsgSend (force_sensor_fd, &msg, sizeof (msg), msg_reply, 255);
    +   if (ret == -1)
    +     {
    + fprintf (stderr, "Unable to MsgSend() to server: %s\n",
    + strerror (errno));
    + return EXIT_FAILURE;
    +     }
    +   printf ("client: msg_reply:\n%s\n", msg_reply);
    +
    +   return 2;
    +}
    +int _set_number_of_force_sensors(int num)
    +{
    +   return 2;
    +}
    +int _read_force_sensor(int id, double *forces)
    +{
    +   for(int i=0;i < 6; i++){
    +      forces[i] = force_data[6*id+i];
    +  }
    +   return 0;
    +}
    +int _read_force_offset(int id, double *offsets)
    +{
    +   return 0;
    +}
    +
    +int _write_force_offset(int id, double *offsets)
    +{
    +   return 0;
    +}
    +//END: For Force Sensor
     
     robot::robot() : m_fzLimitRatio(0), m_maxZmpError(DEFAULT_MAX_ZMP_ERROR), m_calibRequested(false), m_pdgainsFilename("PDgains.sav"), wait_sem(0)
     {
    @@ -64,7 +143,7 @@
     
     
         set_number_of_joints(numJoints());
    -    set_number_of_force_sensors(numSensors(Sensor::FORCE));
    +    _set_number_of_force_sensors(numSensors(Sensor::FORCE));
         set_number_of_gyro_sensors(numSensors(Sensor::RATE_GYRO));
         set_number_of_accelerometers(numSensors(Sensor::ACCELERATION));
     
    @@ -72,12 +151,12 @@
         accel_sum.resize(numSensors(Sensor::ACCELERATION));
     
         if ((number_of_joints() != numJoints())
    -	|| (number_of_force_sensors() != numSensors(Sensor::FORCE))
    +	|| (_number_of_force_sensors() != numSensors(Sensor::FORCE))
     	|| (number_of_gyro_sensors() != numSensors(Sensor::RATE_GYRO))
     	|| (number_of_accelerometers() != numSensors(Sensor::ACCELERATION))){
           std::cerr << "VRML and IOB are inconsistent" << std::endl;
           std::cerr << "  joints:" << numJoints() << "(VRML), " << number_of_joints() << "(IOB)"  << std::endl;
    -      std::cerr << "  force sensor:" << numSensors(Sensor::FORCE) << "(VRML), " << number_of_force_sensors() << "(IOB)"  << std::endl;
    +      std::cerr << "  force sensor:" << numSensors(Sensor::FORCE) << "(VRML), " << _number_of_force_sensors() << "(IOB)"  << std::endl;
           std::cerr << "  gyro sensor:" << numSensors(Sensor::RATE_GYRO) << "(VRML), " << number_of_gyro_sensors() << "(IOB)"  << std::endl;
           std::cerr << "  accelerometer:" << numSensors(Sensor::ACCELERATION) << "(VRML), " << number_of_accelerometers() << "(IOB)"  << std::endl;
           return false;
    @@ -93,9 +172,9 @@
         double force[6], offsets[6];
         
         for (int i=0; i<numSensors(Sensor::FORCE); i++) {
    -        read_force_sensor(i, force);
    +        _read_force_sensor(i, force);
             for (int j=0; j<6; j++) offsets[j] = -force[j];
    -        write_force_offset(i, offsets);
    +        _write_force_offset(i, offsets);
         }
     }
     
    @@ -240,6 +319,22 @@
     void robot::oneStep()
     {
         calibrateInertiaSensorOneStep();
    +    //BEGIN: For force sensor. see https://github.com/start-jsk/rtmros_hironx/pull/253#issuecomment-57050332
    +    int num = 1, ret;
    +    char msg_reply[255];
    +    client_msg_t msg;
    +    msg.msg_no = _IO_MAX + num;
    +    ret = MsgSend (force_sensor_fd, &msg, sizeof (msg), msg_reply, 255);
    +    if (ret == -1)
    +      {
    +        fprintf (stderr, "Unable to MsgSend() to server: %s\n",
    +        strerror (errno));
    +        return ;
    +      }
    +    memcpy (force_data, msg_reply, sizeof(float)*12);
    +
    +    calibrateInertiaSensorOneStep();
    +    //END: For force sensor. see https://github.com/start-jsk/rtmros_hironx/pull/253#issuecomment-57050332
         gain_control();
         if (m_calibRequested){
             ::initializeJointAngle(m_calibJointName.c_str(), 
    @@ -398,7 +493,7 @@
     
     void robot::readForceSensor(unsigned int i_rank, double *o_forces)
     {
    -    read_force_sensor(i_rank, o_forces);
    +    _read_force_sensor(i_rank, o_forces);
     }
     
     void robot::writeJointCommands(const double *i_commands)
    @@ -495,7 +590,7 @@
     
         if (m_rLegForceSensorId >= 0){
             double force[6];
    -        read_force_sensor(m_rLegForceSensorId, force);
    +        _read_force_sensor(m_rLegForceSensorId, force);
             if (force[FZ] > totalMass()*G*m_fzLimitRatio){
     	    std::cerr << time_string() << ": right Fz limit over: Fz = " << force[FZ] << std::endl;
                 o_reason = EMG_FZ;
    @@ -505,7 +600,7 @@
         } 
         if (m_lLegForceSensorId >= 0){
             double force[6];
    -        read_force_sensor(m_lLegForceSensorId, force);
    +        _read_force_sensor(m_lLegForceSensorId, force);
             if (force[FZ] > totalMass()*G*m_fzLimitRatio){
     	    std::cerr << time_string() << ": left Fz limit over: Fz = " << force[FZ] << std::endl;
                 o_reason = EMG_FZ;

Appendix-B. Some files may have been undesirably reverted. So manually modify them back.

 * /opt/jsk/bin/NameServer.sh::

    - /usr/local/bin/omniNames -logdir /tmp -start 15005 > $LOG_DIR/NameServer-$DATESTRING.log 2>&1 &
    + /opt/jsk/bin/omniNames -logdir /tmp -start 15005 > $LOG_DIR/NameServer-$DATESTRING.log 2>&1 &
