/*
 * nextage-open.hpp
 *
 * Copyright (C) 2014 by Kawada Industries, Inc. and Kawada Robotics Corp.
 *
 * Copying and distribution of this file AS IS are permitted.
 * Distribution of any derivative or modified version of this file is NOT permitted.
 */

/**
Together with iob.cpp that resides in the same directory, nextage-open.hpp consists of an opensource interface for the manufactorer-proprietary hardware interface libhrpiob.so. See the <a href = "http://wiki.ros.org/rtmros_nextage/Tutorials/Install%20NEXTAGE%20OPEN%20software%20on%20your%20machine#Overall_software_components">software components diagram</a> for the visual relationship of components.

Despite its name, nextage-open.hpp works for both for the opensource version of Hiro (represented as hironx_ros_bridge package) and NEXTAGE.
 */

#ifndef _NEXTAGE_OPEN_HPP_
#define _NEXTAGE_OPEN_HPP_

#include <unistd.h> // pid_t


namespace NEXTAGE_OPEN
{
    class OpenIFv10; // for ver 1.0
    
    extern "C" bool nextage_open_supported(const char *version);
    extern "C" OpenIFv10 *nextage_open_getIFv10(void);
    
    class OpenIFv10 {
    public:
        virtual ~OpenIFv10() {}
        virtual int open_iob(void) = 0;
        virtual int close_iob(void) = 0;
        
        //virtual int number_of_joints(void) = 0; // hardcoded at iob.cpp for NEXTAGE OPEN,
                                                  // because this might be calld before open_iob().
        virtual int read_actual_angle(int id, double *angle) = 0;
        virtual int read_actual_angles(double *angles) = 0;
        
        virtual int read_power_state(int id, int *s) = 0;
        virtual int write_power_command(int id, int com) = 0;
        virtual int read_power_command(int id, int *com) = 0;
        virtual int read_servo_state(int id, int *s) = 0;
        virtual int read_servo_alarm(int id, int *a) = 0;
        
        virtual int read_command_angle(int id, double *angle) = 0;
        virtual int read_command_angles(double *angles) = 0;
        virtual int write_command_angle(int id, double angle) = 0;
        virtual int write_command_angles(const double *angles) = 0;
        virtual int write_servo(int id, int com) = 0;
        virtual int read_driver_temperature(int id, unsigned char *v) = 0;
        virtual int read_calib_state(int id, int *s) = 0;
        virtual int read_power(double *voltage, double *current) = 0;
        virtual int reset_body(void) = 0;
        virtual int lock_iob(void) = 0;
        virtual int unlock_iob(void) = 0;
        virtual int read_lock_owner(pid_t *pid) = 0;
        virtual unsigned long long read_iob_frame(void) = 0;
        virtual int number_of_substeps(void) = 0;
        virtual int wait_for_iob_signal(void) = 0;
        virtual long get_signal_period(void) = 0;
        
        
        virtual int initializeJointAngle(const char *name, const char *option) = 0;
        virtual int read_digital_input(char *dIn) = 0;
        virtual int length_digital_input(void) = 0;
        virtual int write_digital_output(const char *doutput) = 0;
        virtual int write_digital_output_with_mask(const char *doutput, const char *mask) = 0;
        virtual int length_digital_output(void) = 0;
        virtual int read_digital_output(char *doutput) = 0;
    };

}

#endif // _NEXTAGE_OPEN_HPP_

// Emacs Settings
// Local Variables:
//  mode: c++
//  mode: outline-minor
//  tab-width: 4
//  indent-tabs-mode: nil
//  c-basic-offset: 4
// End:
