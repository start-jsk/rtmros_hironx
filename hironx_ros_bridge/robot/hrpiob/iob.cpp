// -*- Mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
//
// iob.cpp for NEXTAGE OPEN
//
//
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
/*
* iob.cpp
* Copyright (c) 2014, Tokyo Opensource Robotics Kyokai Association
*
* THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
* COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
* COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
* AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
*
* BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
* BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
* CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
* CONDITIONS.
* http://creativecommons.org/licenses/by-nc/4.0
*/

#include <cstdio>
#include <dlfcn.h>

// depended by iob.h
#include <unistd.h> // pid_t, etc.
#include <pthread.h> // mutex

#include <hrpsys/io/iob.h>
#include <nextage-open.hpp>

#define JID_INVALID -2

// private variables
static NEXTAGE_OPEN::OpenIFv10 *nxifv1 = NULL;

#define CHECK_JOINT_ID(id) if((id) < 0 || (id) >= number_of_joints()) return E_ID
#define CHECK_JOINT_ID_DOF(id) if((id) < 0 || (id) >= DOF) return E_ID

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME number_of_joints -> Done
int number_of_joints()
{
	return (15); // this might be called before open_iob(). 
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD number_of_gyro_sensors - Done
int set_number_of_joints(int num)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD number_of_force_sensors - Done
int number_of_force_sensors()
{
    return 0;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD set_number_of_force_sensors - Done
int set_number_of_force_sensors(int num)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD number_of_gyro_sensors - Done
int number_of_gyro_sensors()
{
    return 0;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD set_number_of_gyro_sensors - Done
int set_number_of_gyro_sensors(int num)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD number_of_accelerometers - Done
int number_of_accelerometers()
{
    return 0;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD set_number_of_accelerometers - Done
int set_number_of_accelerometers(int num)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD number_of_attitude_sensors - Done
int number_of_attitude_sensors()
{
    return 0;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_actual_angle - TODO dynamic test
//int readActualAngle(const uint32_t jointid, double *o_angle)
int read_actual_angle(int id, double *angle)
{
	if (nxifv1)
		return nxifv1->read_actual_angle(id, angle);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_actual_angles - TODO dynamic check
int read_actual_angles(double *angles)
{
	if (nxifv1)
		return nxifv1->read_actual_angles(angles);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_angle_offset - Done
int read_angle_offset(int id, double *angle)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD write_angle_offset - Done
int write_angle_offset(int id, double angle)
{
    return FALSE;
}



/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME: read_power_state - Done, TODO static test
int read_power_state(int id, int *s)
{
	if (nxifv1)
		return nxifv1->read_power_state(id, s);
    else
		return FALSE;
}


// RENAME write_power_command - Done. TODO dynamic test
int write_power_command(int id, int com)
{
	if (nxifv1)
		return nxifv1->write_power_command(id, com);
    else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_power_command - done, TODO dynamic
int read_power_command(int id, int *com)
{
	if (nxifv1)
		return nxifv1->read_power_command(id, com);
    else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_servo_state - TODO dynamic test
int read_servo_state(int id, int *s)
{
	if (nxifv1)
		return nxifv1->read_servo_state(id, s);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_servo_alarm - done, TODO dynamic
int read_servo_alarm(int id, int *a)
{
	if (nxifv1)
		return nxifv1->read_servo_alarm(id, a);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_control_mode - done, TODO static test
int read_control_mode(int id, joint_control_mode *s)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD write_control_mode - done
int write_control_mode(int id, joint_control_mode s)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_actual_torque - done
int read_actual_torque(int id, double *angle)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME: read_actual_torques
int read_actual_torques(double *torques)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_command_torque - done
int read_command_torque(int id, double *torque)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD write_command_torque - done
int write_command_torque(int id, double torque)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_command_torques - done
int read_command_torques(double *torques)
{
    return FALSE;
}

int write_command_torques(const double *torques)
{
    return FALSE;
}


/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_command_angle - done, TODO dynamic
int read_command_angle(int id, double *angle)
{
	if (nxifv1)
		return nxifv1->read_command_angle(id, angle);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_command_angles - done, TODO dynamic
int read_command_angles(double *angles)
{
	if (nxifv1)
		return nxifv1->read_command_angles(angles);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME write_command_angle - done. TODO dynamic
int write_command_angle(int id, double angle)
{
	if (nxifv1)
		return nxifv1->write_command_angle(id, angle);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME write_command_angles - done. TODO dynamic
int write_command_angles(const double *angles)
{
	if (nxifv1)
		return nxifv1->write_command_angles(angles);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_pgain - done. TODO static
int read_pgain(int id, double *gain)
{
    return E_ID; // FALSE ?
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME write_pgain - requires implementation
int write_pgain(int id, double gain)
{
    return E_ID; // FALSE ?
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_dgain - done
int read_dgain(int id, double *gain)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME write_dgain - done
//int
int write_dgain(int id, double gain)
{
    return E_ID;
}

// WANT read_igain
// WANT write_igain

int read_actual_velocity(int id, double *vel)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_command_velocity - done
int read_command_velocity(int id, double *vel)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD write_command_velocity - done
int write_command_velocity(int id, double vel)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_command_velocities - done
int read_command_velocities(double *vels)
{
    return FALSE;
}

int read_actual_velocities(double *vels)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD write_command_velocities - done
int write_command_velocities(const double *vels)
{
    return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME write_servo TODO dynamic test
int write_servo(int id, int com)
{
	if (nxifv1)
		return nxifv1->write_servo(id, com);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_driver_temperature - done
int read_driver_temperature(int id, unsigned char *v)
{
	if (nxifv1)
		return nxifv1->read_driver_temperature(id, v);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_calib_state - done. TODO dynamic
int read_calib_state(int id, int *s)
{
	if (nxifv1)
		return nxifv1->read_calib_state(id, s);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD length_of_extra_servo_state - TODO implementation
size_t length_of_extra_servo_state(int id)
{
    return 0;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_extra_servo_state - TODO implementation
int read_extra_servo_state(int id, int *state)
{
    return TRUE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_force_sensor - done
int read_force_sensor(int id, double *forces)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// Add read_force_offset - done
int read_force_offset(int id, double *offsets)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD write_force_offset - done
int write_force_offset(int id, double *offsets)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_gyro_sensor - done
//int
int read_gyro_sensor(int id, double *rates)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_gyro_sensor_offset - done
int read_gyro_sensor_offset(int id, double *offset)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD write_gyro_sensor_offset - done
int write_gyro_sensor_offset(int id, double *offset)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_accelerometer - done
//int
int read_accelerometer(int id, double *accels)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_accelerometer_offset - done
int read_accelerometer_offset(int id, double *offset)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD write_accelerometer_offset - done
int write_accelerometer_offset(int id, double *offset)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_attitude_sensor - done
int read_attitude_sensor(int id, double *att)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD write_attitude_sensor_offset - done
int write_attitude_sensor_offset(int id, double *offset)
{
    return E_ID;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// ADD read_power - done. TODO implement
int read_power(double *voltage, double *current)
{
	if (nxifv1)
		return nxifv1->read_power(voltage, current);
	else
		return FALSE;
}

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// RENAME read_driver_temperature - done
//int
int read_temperature(int id, double *v)
{
    return E_ID;
}


static pthread_mutex_t openLock = PTHREAD_MUTEX_INITIALIZER;

// RENAME open_iob - done TODO check implementation
int open_iob(void)
{
	pthread_mutex_lock (&openLock);
	if (nxifv1 == NULL) {
		void *lib = dlopen("libNxOpenCore.so", RTLD_LOCAL|RTLD_NOW);
		if (lib == NULL) {
			std::fprintf(stdout, "** failed in dlopen(libNxOpenCore.so): %s\n", dlerror());
			pthread_mutex_unlock(&openLock);
			return FALSE;
		}

		bool (*check)(const char *);
		check = reinterpret_cast<bool (*)(const char *)>(dlsym(lib, "nextage_open_supported"));
		if (check==NULL) {
			std::fprintf(stdout, "** failed to find nextage_open_supported(): %s\n", dlerror());
			pthread_mutex_unlock(&openLock);
			return FALSE;
		}

		if (check("v1.0")==false) {
			std::fprintf(stdout, "** v1.0 not supported\n");
			pthread_mutex_unlock(&openLock);
			return FALSE;
		}

		NEXTAGE_OPEN::OpenIFv10 * (*getif)(void);
		getif = reinterpret_cast<NEXTAGE_OPEN::OpenIFv10 * (*)(void)>(dlsym(lib, "nextage_open_getIFv10"));
		if (getif==NULL) {
			std::fprintf(stdout, "** failed to find nextage_open_getIFv10(): %s\n", dlerror());
			pthread_mutex_unlock(&openLock);
			return FALSE;
		}

		nxifv1 = getif();
		if (nxifv1 == NULL) {
			std::fprintf(stdout, "** failed to getIF()\n");
			pthread_mutex_unlock(&openLock);
			return FALSE;
		}


		std::fprintf(stdout, "open_iob - NEXTAGE OPEN  I/F v1 instance at 0x%x\n", nxifv1);
		pthread_mutex_unlock(&openLock);
        return nxifv1->open_iob();

    } else {
		std::fprintf(stdout, "open_iob - NEXTAGE OPEN I/F instance is not NULL, ignored\n");
		pthread_mutex_unlock(&openLock);
		return TRUE;
	}

	pthread_mutex_unlock(&openLock);
    return FALSE; // unexpected
}

// RENAME close_iob - done. TODO check implementation
//int
int close_iob(void)
{
	// do nothing
	return TRUE;
#if 0
	if (nxifv1 != NULL) {
		int res=nxifv1->close_iob();
		delete nxifv1;
		nxifv1 = NULL;
		return res;
	} else {
		std::fprintf(stdout, "close_iob(): nxifv1 is NULL\n");
	}

    return FALSE;
#endif
}


// ADD reset_body - TODO static test
int reset_body(void)
{
	if (nxifv1) 
		return nxifv1->reset_body();
	else
		return FALSE;
}

// RENAME: lock_iob - done. TODO check implementation
int lock_iob()
{
	if (nxifv1) 
		return nxifv1->lock_iob();
	else
		return FALSE;
}

// RENAME: unlock_iob - done. TODO check static test
//int
//ShMIF::unlock ()
int unlock_iob()
{
	if (nxifv1) 
		return nxifv1->unlock_iob();
	else
		return FALSE;
}

// ADD read_lock_owner - done. TODO static test
int read_lock_owner(pid_t *pid)
{
	if (nxifv1) 
		return nxifv1->read_lock_owner(pid);
	else
		return FALSE;
}

// RENAME read_iob_frame - done. TODO static test
//WANT uint64_t read_iob_frame()
//int read_iob_frame()
unsigned long long read_iob_frame()
{
	if (nxifv1) 
		return nxifv1->read_iob_frame();
	else
		return FALSE;
}

// REMOVE getAddr

// RENAME number_of_substeps
int number_of_substeps()
{
	if (nxifv1) 
		return nxifv1->number_of_substeps();
	else
		return FALSE;
    //return 1;
    //return 5;
}

// RENAME wait_for_iob_signal - done. TODO implementation
int wait_for_iob_signal()
{
	if (nxifv1) 
		return nxifv1->wait_for_iob_signal();
	else
		return FALSE;
}

/**
 * @brief set the period of signals issued by wait_for_iob_signal()
 * @param period_ns the period of signals[ns]
 * @return TRUE if set successfully, FALSE otherwise
 */
int set_signal_period(long period_ns)
{
    return FALSE;
}

/**
 * @brief get the period of signals issued by wait_for_iob_signal()
 * @return the period of signals[ns]
 */
long get_signal_period()
{
	if (nxifv1) 
		return nxifv1->get_signal_period();
	else
		return FALSE;
    //return 5 * 1e6;
}



/////////////////////////////////////////////////
// TODO move internal

/*
 *  for safety
 */
// TODO everything to do with calibrateJoints is to be moved here
// RENAME initializeJointAngle
int initializeJointAngle(const char *name, const char *option)
{
	if (nxifv1)
		return nxifv1->initializeJointAngle(name, option);
	else
		return FALSE;
}

int read_digital_input(char *dIn)
{
	if (nxifv1)
		return nxifv1->read_digital_input(dIn);
	else
		return FALSE;
}

int length_digital_input()
{
	if (nxifv1)
		return nxifv1->length_digital_input();
	else
		return FALSE;
}

int write_digital_output(const char *doutput)
{
	if (nxifv1)
		return nxifv1->write_digital_output(doutput);
	else
		return FALSE;
}

int write_digital_output_with_mask(const char *doutput, const char *mask)
{
	if (nxifv1)
		return nxifv1->write_digital_output_with_mask(doutput, mask);
	else
		return FALSE;
}

int length_digital_output()
{
	if (nxifv1)
		return nxifv1->length_digital_output();
	else
		return FALSE;
}

int read_digital_output(char *doutput)
{
	if (nxifv1)
		return nxifv1->read_digital_output(doutput);
	else
		return FALSE;
}
