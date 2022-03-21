/**
 * @file test_VL531X.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include "vl53l1x.h"
#include <robotcontrol.h>

extern uint8_t i2c_bus;

int main()
{
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();
	
	VL53L1_Dev_t Device;
	uint8_t addr = VL53L1X_DEFAULT_DEVICE_ADDRESS;
	uint8_t i2cbus = 1;
	int16_t status = 0;
	uint16_t rtn;

	status = VL53L1X_InitDriver(&Device, i2cbus, addr);
	if(status!=0){
		printf("ERROR: VL53LX Not Responding\n");
		return -1;;
	}
	rc_usleep(1E4);
	printf("Initializing Sensor...\n");
	VL53L1X_SensorInit(&Device);
	rc_usleep(1E4);
	//VL53L1X_SetDistanceMode(&Device,2);
	//VL53L1X_SetInterMeasurementInMs(&Device,200);
	//VL53L1X_SetTimingBudgetInMs(&Device,100);

	rc_usleep(1E4);
	VL53L1X_GetDistanceMode(&Device,&rtn);
	printf("Distance Mode: %d\n", rtn);

	VL53L1X_GetInterMeasurementInMs(&Device,&rtn);
	printf("Measurement Period: %dms\n", rtn);
	uint16_t rate = rtn;

	VL53L1X_GetTimingBudgetInMs(&Device,&rtn);
	printf("Timing Budget: %dms\n", rtn);

	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	uint16_t distance = 0;
	uint8_t tmp = 0;
	VL53L1X_StartRanging(&Device);
	while(rc_get_state()!=EXITING){
		rc_usleep(1000*rate);
		while (tmp == 0){
			status = VL53L1X_CheckForDataReady(&Device, &tmp);
			rc_usleep(1E2);
		}
		tmp = 0;
		VL53L1X_ClearInterrupt(&Device);
		VL53L1X_GetDistance(&Device, &distance);
		printf("Distance: %fm \r", distance/1000.0);
		fflush(stdout);
		// always sleep at some point	
	}
	VL53L1X_StopRanging(&Device);
	// turn off LEDs and close file descriptors
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}