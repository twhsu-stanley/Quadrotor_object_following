/**
 * @file rc_log_mpu.c
 * @example    rc_log_mpu
 * @brief      log data from the mpu to a csv file
 */

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <getopt.h>
#include <rc/mpu.h>
#include <rc/time.h>
#include <inttypes.h>

// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2

static int enable_magnetometer = 0;
static int enable_thermometer = 0;
static int enable_warnings = 0;
static int running = 0;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}

int main(int argc, char *argv[])
{
        printf("File name: %s.csv\n", argv[1]);
        char *file_name = strcat(argv[1],".csv");
        FILE *fp = fopen (file_name, "w+");
        if (fp == NULL)
        {
                fprintf(stderr, "\nError opened file\n");
        }
        
        rc_mpu_data_t data; //struct to hold new data

        // set signal handler so the loop can exit cleanly
        signal(SIGINT, __signal_handler);
        running = 1;
        // use defaults for now, except also enable magnetometer.
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
        conf.enable_magnetometer = enable_magnetometer;
        conf.show_warnings = enable_warnings;
        conf.dmp_sample_rate = 200;
        if(rc_mpu_initialize(&data, conf)){
                fprintf(stderr,"rc_mpu_initialize_failed\n");
                return -1;
        }

        // print the header
        printf("   Accel XYZ(m/s^2)  |");
        printf("   Gyro XYZ (deg/s)  |");
        fprintf(fp,"Time (ns), Acc X, Acc Y, Acc Z, Gyro X,  Gyro Y, Gyro Z\n");
        printf("\n");

        while (running) {
                printf("\r");
                // read sensor data
                if(rc_mpu_read_accel(&data)<0){
                        printf("read accel data failed\n");
                }
                if(rc_mpu_read_gyro(&data)<0){
                        printf("read gyro data failed\n");
                }
                if(enable_magnetometer && rc_mpu_read_mag(&data)){
                        printf("read mag data failed\n");
                }
                if(enable_thermometer && rc_mpu_read_temp(&data)){
                        printf("read imu thermometer failed\n");
                }

                printf("%6.2f %6.2f %6.2f |", data.accel[0],data.accel[1],data.accel[2]);
                printf("%6.2f %6.2f %6.2f |", data.gyro[0],data.gyro[1], data.gyro[2]);
                fflush(stdout);
                
                // print data to file
                fprintf(fp," %" PRId64 ", %6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f\n",rc_nanos_since_epoch(), data.accel[0],data.accel[1],data.accel[2],data.gyro[0],data.gyro[1],data.gyro[2]); // printing to file
        }

        // close file
        fclose (fp);

        printf("\n");
        rc_mpu_power_off();
        return 0;
}
