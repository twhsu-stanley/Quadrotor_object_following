//Timers Prince is using to check how long are running


/**
 * @file main.c
 *
 * @brief       Initialize all submodules and librobotcontrol suite
 *
 * see README.txt for description and use
 **/

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <rc/adc.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/cpu.h>
#include <rc/dsm.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>

// #include <vl53l1x.h>
#include <input_manager.h>
#include <log_manager.h>
#include <mix.h>
#include <naza_gps.h>
#include <path.h>
#include <printf_manager.h>
#include <setpoint_manager.h>
#include <settings.h>  // contains extern settings variable
#include <state_estimator.h>
#include <state_machine.h>
#include <thrust_map.h>
#include <xbee_receive.h>
#include <image_receive.h> // for lcm (image data)
#include <pni_rm3100.h>
#include <benchmark.h>
#include <ntp_read.h>
#include <realsense_payload_receive.h>
// #include <lcm/lcm.h>
// #include "./../../lcmtypes/pose_xyt_t.h"
// #include "./../../lcmtypes/mbot_encoder_t.h"
// #include "./../../lcmtypes/mbot_imu_t.h"
// #include "./../../lcmtypes/mbot_motor_command_t.h"
// #include "./../../lcmtypes/odometry_t.h"
// #include "./../../lcmtypes/oled_message_t.h"
// #include "./../../lcmtypes/timestamp_t.h"
// #include "./../../lcmtypes/reset_odometry_t.h"
#include <socket_manager.h>


// LCM Channel Names - should not be changed
// #define TRUE_POSE_CHANNEL           "TRUE_POSE"
// #define ODOMETRY_CHANNEL            "ODOMETRY"
// #define RESET_ODOMETRY_CHANNEL      "RESET_ODOMETRY"
// #define CONTROLLER_PATH_CHANNEL     "CONTROLLER_PATH"
// #define MBOT_IMU_CHANNEL            "MBOT_IMU"
// #define MBOT_ENCODER_CHANNEL        "MBOT_ENCODERS"
// #define MBOT_MOTOR_COMMAND_CHANNEL  "MBOT_MOTOR_COMMAND"
// #define MBOT_TIMESYNC_CHANNEL       "MBOT_TIMESYNC"
// #define OBJ_POSITION_VISION_CHANNEL                "OBJ_POS_VISION"
// // #define LCM_ADDRESS                 "udpm://239.255.76.67:7667?ttl=1"
// #define LCM_ADDRESS                 "udpm://239.255.76.67:7667?ttl=1"


//LCM handler functions
// void motor_command_handler(const lcm_recv_buf_t *rbuf, 
//                                   const char *channel,
//                                   const mbot_motor_command_t *msg, 
//                                   void *user);

// void timesync_handler(const lcm_recv_buf_t * rbuf, 
//                              const char *channel,
//                              const timestamp_t *timestamp, 
//                              void *_user);

// void obj_loc_in_vision_handler(const lcm_recv_buf_t * rbuf, 
//                              const char *channel,
//                              const pose_xyt_t *msg, 
//                              void *_user);

// lcm_t * lcm;
/**
 *  @brief      Standard exit for initialization failures
 */
#define FAIL(str)                       \
    fprintf(stderr, str);               \
    rc_led_set(RC_LED_GREEN, 0);        \
    rc_led_blink(RC_LED_RED, 8.0, 2.0); \
    return -1;

/**
 *  @brief      Prints main function commond line ussage (arguments)
 */
void print_usage()
{
    printf("\n");
    printf(" Options\n");
    printf(" -s\t\t<settings file> Specify settings file to use\n");
    printf(" -h\t\tPrint this help message\n");
    printf("\n");
    printf("Some example settings files are included with the\n");
    printf("source code. You must specify the location of one of these\n");
    printf("files or ideally the location of your own settings file.\n");
    printf("\n");
}

/**
 * @brief       temporary check for dsm calibration until I add this to librobotcontrol
 */
static int __rc_dsm_is_calibrated()
{
    if (!access("/var/lib/robotcontrol/dsm.cal", F_OK))
        return 1;
    else
        return 0;
}

/**
 * @brief       If the user holds the pause button for 2 seconds, set state to exiting which
 * triggers the rest of the program to exit cleanly.
 */
void on_pause_press()
{
    int i = 0;
    const int quit_check_us = 100000;
    const int samples = 2000000 / quit_check_us;

    // toggle betewen paused and running modes
    if (rc_get_state() == RUNNING)
    {
        rc_set_state(PAUSED);
        printf("PAUSED\n");
    }
    else if (rc_get_state() == PAUSED)
    {
        rc_set_state(RUNNING);
        printf("RUNNING\n");
    }
    fflush(stdout);

    // now keep checking to see if the button is still held down
    for (i = 0; i < samples; i++)
    {
        rc_usleep(quit_check_us);
        if (rc_button_get_state(RC_BTN_PIN_PAUSE) == RC_BTN_STATE_RELEASED)
        {
            return;
        }
    }
    printf("long press detected, shutting down\n");
    rc_set_state(EXITING);
    return;
}

/**
 * @brief      Interrupt service routine for IMU
 *
 * This is called every time the Invensense IMU has new data
 */
static void __imu_isr(void)
{
    // record time of imu interupt
    state_estimate.imu_time_ns = rc_nanos_since_epoch();
    if(settings.log_benchmark) benchmark_timers.tIMU = rc_nanos_since_epoch();
    
    //Get XBee data
    XBEE_getData();
    if(settings.log_benchmark) benchmark_timers.tXBEE = rc_nanos_since_epoch();

    // Get Realsense Payload Data
    switch (REALSENSE_getData()) {
        case 0:
            // No New Message
            break;
        case 1:
            // New Pose Update Message
            break;
        case 2:
            // New Landing Command Message
            plan_rsp_landing();
            break;
        default:
            break;
    }

    //Update the state machine
    sm_transition(&waypoint_state_machine, xbeeMsg.sm_event);
    if(settings.log_benchmark) benchmark_timers.tSM = rc_nanos_since_epoch();

    //Read from GPS sensor
    gps_getData();
    if(settings.log_benchmark) benchmark_timers.tGPS = rc_nanos_since_epoch();

    //Read from PNI RM3100 Magnetometer (skip if readings aren't used)
    if((settings.printf_rm3100 || settings.log_rm3100))
    {
        rm3100_read_meas(&rm3100_data);
    }    
    if(settings.log_benchmark) benchmark_timers.tPNI = rc_nanos_since_epoch();

    //Navigation
    state_estimator_march();
    if(settings.log_benchmark) benchmark_timers.tNAV = rc_nanos_since_epoch();
    
    //Guidance
    setpoint_manager_update();
    if(settings.log_benchmark) benchmark_timers.tGUI = rc_nanos_since_epoch();

    //Control
    feedback_march();
    if(settings.log_benchmark) benchmark_timers.tCTR = rc_nanos_since_epoch();

    //Save data to log file
    if (settings.enable_logging) log_manager_add_new();
    if(settings.log_benchmark) benchmark_timers.tLOG = rc_nanos_since_epoch();

    //Currently, this only reads from the BMP pressure sensor
    state_estimator_jobs_after_feedback();
    if(settings.log_benchmark) benchmark_timers.tIMU_END = rc_nanos_since_epoch();
}

void* lcm_subscribe_loop(void* ptr);

/**
 * @brief       Initialize the IMU, start all the threads, and wait until something triggers
 * a shut down by setting the RC state to EXITING.
 *
 * @return     0 on success, -1 on failure
 */
int main(int argc, char* argv[])
{
    ntpCounter = NTP_COUNTER_THRESH; //Start here so that we check offset first trip through imu_isr()

    int c;
    char* settings_file_path = "../settings/quad_settings.json";
    // lcm_t* lcm;
    // parse arguments
    opterr = 0;
    while ((c = getopt(argc, argv, "s:w:h")) != -1)
    {
        switch (c)
        {
            // settings file option
            case 's':
                settings_file_path = optarg;
                printf("User specified settings file:\n%s\n", settings_file_path);
                break;

            // help mode
            case 'h':
                print_usage();
                return 0;

            default:
                printf("\nInvalid Argument \n");
                print_usage();
                return -1;
        }
    }

    // settings file option is mandatory
    if (settings_file_path == NULL)
    {
        print_usage();
        return -1;
    }

    // first things first, load settings which may be used during startup
    if (settings_load_from_file(settings_file_path) < 0)
    {
        fprintf(stderr, "ERROR: failed to load settings\n");
        return -1;
    }
    printf("Loaded settings: %s\n", settings.name);

    // before touching hardware, make sure another instance isn't running
    // return value -3 means a root process is running and we need more
    // privileges to stop it.
    if (rc_kill_existing_process(2.0) == -3) return -1;

    // start with both LEDs off
    if (rc_led_set(RC_LED_GREEN, 0) == -1)
    {
        fprintf(stderr, "ERROR in main(), failed to set RC_LED_GREEN\n");
        return -1;
    }
    if (rc_led_set(RC_LED_RED, 0) == -1)
    {
        fprintf(stderr, "ERROR in main() failed to set RC_LED_RED\n");
        return -1;
    }

    // make sure IMU is calibrated
    if (!rc_mpu_is_gyro_calibrated())
    {
        FAIL("ERROR: must calibrate gyroscope with rc_calibrate_gyro first\n")
    }
    if (!rc_mpu_is_accel_calibrated())
    {
        FAIL("ERROR: must calibrate accelerometer with rc_calibrate_accel first\n")
    }
    if (settings.enable_magnetometer && !rc_mpu_is_gyro_calibrated())
    {
        FAIL("ERROR: must calibrate magnetometer with rc_calibrate_mag first\n")
    }
    if (!__rc_dsm_is_calibrated())
    {
        FAIL("ERROR: must calibrate DSM with rc_calibrate_dsm first\n")
    }

    // turn cpu freq to max for most consistent performance and lowest
    // latency servicing the IMU's interrupt service routine
    // this also serves as an initial check for root access which is needed
    // by the PRU later. PRU root acces might get resolved in the future.
    if (rc_cpu_set_governor(RC_GOV_PERFORMANCE) < 0)
    {
        FAIL("ERROR: can't set CPU governor, need to run as root\n")
    }

    // do initialization not involving threads
    printf("initializing thrust map\n");
    if (thrust_map_init(settings.thrust_map) < 0)
    {
        FAIL("ERROR: failed to initialize thrust map\n")
    }
    printf("initializing mixing matrix\n");
    if (mix_init(settings.layout) < 0)
    {
        FAIL("ERROR: failed to initialize mixing matrix\n")
    }
    printf("initializing setpoint_manager\n");
    if (setpoint_manager_init() < 0)
    {
        FAIL("ERROR: failed to initialize setpoint_manager\n")
    }

    // initialize cape hardware, this prints an error itself if unsuccessful
    printf("initializing servos\n");
    if (rc_servo_init() == -1)
    {
        FAIL("ERROR: failed to initialize servos, probably need to run as root\n")
    }
    printf("initializing adc\n");
    if (rc_adc_init() == -1)
    {
        FAIL("ERROR: failed to initialize ADC")
    }

    // start signal handler so threads can exit cleanly
    printf("initializing signal handler\n");
    if (rc_enable_signal_handler() < 0)
    {
        FAIL("ERROR: failed to complete rc_enable_signal_handler\n")
    }

    // start threads
    printf("initializing DSM and input_manager\n");
    if (input_manager_init() < 0)
    {
        FAIL("ERROR: failed to initialize input_manager\n")
    }

    printf("initializing object detection socket\n");
    if (socketserver_init() < 0)
    {
        FAIL("ERROR: failed to initialize socket_manager\n")
    }
	// start lcm handle thread
	// printf("starting lcm thread... \n");
	// lcm = lcm_create(LCM_ADDRESS);
	// pthread_t lcm_subscribe_thread;
    // rc_pthread_create(&lcm_subscribe_thread, lcm_subscribe_loop, (void*) NULL, SCHED_FIFO, LCM_PRIORITY);

    // Initialize alimeter
    // printf("Initializing the alitimeter... \n");
	// uint8_t addr = VL53L1X_DEFAULT_DEVICE_ADDRESS;
	// uint8_t i2cbus = 1;
	// int16_t status = 0;
	// uint16_t rtn;

    // status = VL53L1X_InitDriver(&Device, i2cbus, addr);
	// if(status!=0){
	// 	printf("ERROR: VL53LX Not Responding\n");
	// 	return -1;;
	// }

    // VL53L1X_SensorInit(&Device);

    // VL53L1X_GetDistanceMode(&Device,&rtn);

    // printf("Altimeter Distance Mode: %d\n", rtn);

    // VL53L1X_GetInterMeasurementInMs(&Device,&rtn);
	// printf("Measurement Period: %dms\n", rtn);
	// uint16_t rate = rtn;

	// VL53L1X_GetTimingBudgetInMs(&Device,&rtn);
	// printf("Timing Budget: %dms\n", rtn);
    // VL53L1X_StartRanging(&Device); 

    // initialize buttons and Assign functions to be called when button
    // events occur
    if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US))
    {
        FAIL("ERROR: failed to init buttons\n")
    }
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE, on_pause_press, NULL);

    // initialize log_manager if enabled in settings
    if (settings.enable_logging)
    {
        if (!settings.log_only_while_armed)
        {
            printf("initializing log manager\n");
            if (log_manager_init() < 0)
            {
                FAIL("ERROR: failed to initialize log manager\n")
            }
        }
    }

    // start barometer, must do before starting state estimator
    printf("initializing Barometer\n");
    if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16))
    {
        FAIL("ERROR: failed to initialize barometer\n")
    }

    // set up state estimator
    printf("initializing state_estimator\n");
    if (state_estimator_init() < 0)
    {
        FAIL("ERROR: failed to init state_estimator")
    }

    // set up XBEE serial link
    printf("initializing xbee serial link:%s.\n", settings.xbee_serial_port);
    if (XBEE_init(settings.xbee_serial_port) < 0)
    {
        FAIL("ERROR: failed to init xbee serial link")
    }

    // set up Realsense Payload serial link
    printf("initializing realsense payload serial link:%s.\n", settings.realsense_payload_serial_port);
    if (REALSENSE_init(settings.realsense_payload_serial_port) < 0)
    {
        FAIL("ERROR: failed to init realsense payload serial link")
    }

    printf("initializing gps serial link\n");
    if (gps_init() < 0)
    {
        FAIL("ERROR: failed to init gps serial link");
    }

    //Only initialize PNI RM3100 if we're trying to log or print rm3100 data
    if( (settings.printf_rm3100 || settings.log_rm3100))
    {
        printf("initializing RM3100 Magnetometer\n");
        if (rm3100_init(rm3100_default_config()) < 0)
        {
            FAIL("ERROR: failed to init RM3100 Magnetometer");
        }
    }

    //Read NTP offset once at the start of the program
    ntp_data.ntp_offset_ms = NTP_ERR_NUM;
    ntp_data.ntp_time_updated_ns = 0;
    if (settings.log_ntp)
    {   
        printf("reading initial NTP value...");
        double temp = get_ntp_offset();
        if(temp == NTP_ERR_NUM)
        {
            FAIL("ERROR: Unable to read NTP offset. Ensure NTP server is running and 'ntpq -p' has reasonable outputs\n");
        }
        else
        {
            ntp_data.ntp_offset_ms = temp;
            ntp_data.ntp_time_updated_ns = rc_nanos_since_epoch();
        }        
        printf("offset = %lf ms\n", ntp_data.ntp_offset_ms);
        // rc_usleep(1000000);
    }

    //Initialize waypoint state machine
    printf("initializing waypoint state machine\n");
    if (sm_init(&waypoint_state_machine) < 0)
    {
        FAIL("ERROR: failed to init waypoint state machine");
    }

    // set up feedback controller
    printf("initializing feedback controller\n");
    if (feedback_init() < 0)
    {
        FAIL("ERROR: failed to init feedback controller")
    }

    // start the IMU
    rc_mpu_config_t mpu_conf = rc_mpu_default_config();
    mpu_conf.i2c_bus = I2C_BUS;
    mpu_conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    mpu_conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    mpu_conf.dmp_sample_rate = FEEDBACK_HZ;
    mpu_conf.dmp_fetch_accel_gyro = 1;
    // mpu_conf.orient = ORIENTATION_Z_UP;
    mpu_conf.dmp_interrupt_sched_policy = SCHED_FIFO;
    mpu_conf.dmp_interrupt_priority = IMU_PRIORITY;

    // optionally enbale magnetometer
    mpu_conf.enable_magnetometer = settings.enable_magnetometer;

    // now set up the imu for dmp interrupt operation
    printf("initializing MPU\n");
    if (rc_mpu_initialize_dmp(&mpu_data, mpu_conf))
    {
        FAIL("ERROR: failed to start MPU DMP\n");
    }

    // final setup
    if (rc_make_pid_file() != 0)
    {
        FAIL("ERROR: failed to make a PID file\n")
    }

    // make sure everything is disarmed them start the ISR
    feedback_disarm();
    printf("waiting for dmp to settle...\n");
    fflush(stdout);
    rc_usleep(3000000);
    if (rc_mpu_set_dmp_callback(__imu_isr) != 0)
    {
        FAIL("ERROR: failed to set dmp callback function\n")
    }

    // start printf_thread if running from a terminal
    // if it was started as a background process then don't bother

    if (isatty(fileno(stdout)))
    {
        printf("initializing printf manager\n");
        if (printf_init() < 0)
        {
            FAIL("ERROR: failed to initialize printf_manager\n")
        }
    }

    // set state to running and chill until something exits the program
    rc_set_state(RUNNING);
    while (rc_get_state() != EXITING)
    {
        usleep(50000);
    }

    //Read final NTP offst value
    if (settings.log_ntp)
    {
        double temp = get_ntp_offset();
        if(temp == NTP_ERR_NUM)
        {
            fprintf(stderr, "WARNING: Could not obtain final NTP offset value");
        }
        ntp_data.ntp_offset_ms = temp;
        ntp_data.ntp_time_updated_ns = rc_nanos_since_epoch();
        fprintf(stdout, "Final NTP offset Value %lf\n", ntp_data.ntp_offset_ms);

        if (settings.enable_logging) log_manager_add_new();
    }
    

    // some of these, like printf_manager and log_manager, have cleanup
    // functions that can be called even if not being used. So just call all
    // cleanup functions here.
    printf("cleaning up\n");
    // rc_pthread_timed_join(lcm_subscribe_thread, NULL, 1.5);
    rc_mpu_power_off();
    feedback_cleanup();
    input_manager_cleanup();
    setpoint_manager_cleanup();
    printf_cleanup();
    log_manager_cleanup();
    socketserver_cleanup();
    path_cleanup();
    // VL53L1X_StopRanging(&Device);
    
    // turn off red LED and blink green to say shut down was safe
    rc_led_set(RC_LED_RED, 0);
    rc_led_blink(RC_LED_GREEN, 8.0, 2.0);
    return 0;
}



// /*******************************************************************************
// * lcm_subscribe_loop()
// *
// * thread subscribes to lcm channels and sets handler functions
// * then handles lcm messages in a non-blocking fashion
// *
// * TODO: Add other subscriptions as needed
// *******************************************************************************/
// void *lcm_subscribe_loop(void *data){
//     // pass in lcm object instance, channel from which to read from
//     // function to call when data receiver over the channel,
//     // and the lcm instance again?
//     printf("entering subscribe");
//     pose_xyt_t_subscribe(lcm,
// 						  OBJ_POSITION_VISION_CHANNEL,
// 						  obj_loc_in_vision_handler,
// 						  NULL);


//     // mbot_motor_command_t_subscribe(lcm,
//     // 							   MBOT_MOTOR_COMMAND_CHANNEL,
//     // 							   motor_command_handler,
//     // 							   NULL);

// 	// timestamp_t_subscribe(lcm,
// 	// 					  MBOT_TIMESYNC_CHANNEL,
// 	// 					  timesync_handler,
// 	// 					  NULL);

//     // reset_odometry_t_subscribe(lcm,
//     //                       RESET_ODOMETRY_CHANNEL,
//     //                       reset_odometry_handler,
//     //                       NULL);

//     while(1){
//         // define a timeout (for erroring out) and the delay time
//         lcm_handle_timeout(lcm, 1);
//         rc_nanosleep(1E9 / LCM_HZ);
//     }
//     lcm_destroy(lcm);
//     return 0;
// }

// /*******************************************************************************
// *  motor_pwm_handler()
// *
// *  sets motor PWMS from incoming lcm message
// *
// *******************************************************************************/
// void obj_loc_in_vision_handler(const lcm_recv_buf_t * rbuf, 
//                              const char *channel,
//                              const pose_xyt_t *msg, 
//                              void *_user){

//     printf("Object Location on iphone: %f | %f ", msg->x, msg->y);
    
//     Image_data.u = msg->x;
//     Image_data.v = msg->y;
//     // Image_data.range;
//     // Image_data.bearing;

//     // rc_motor_set(1,mot_l_pol * msg->left_motor_pwm);
//     // rc_motor_set(2,mot_r_pol * msg->right_motor_pwm);
//     // publish_encoder_msg();
//     // watchdog_timer = 0.0;
//     }


    