/**
 * @file state_estimator.c
 *
 */

#include <math.h>
#include <stdio.h>

#include <rc/adc.h>
#include <rc/bmp.h>
#include <rc/led.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/matrix.h>
#include <rc/math/other.h>
#include <rc/math/quaternion.h>
#include <rc/math/algebra.h>
#include <rc/mpu.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <naza_gps.h>
#include <rc_pilot_defs.h>
#include <settings.h>
#include <state_estimator.h>
#include <xbee_receive.h>
#include <socket_manager.h>
#include <vl53l1x.h>
#include <xbee_receive.h>
#include <unistd.h>

#define TWO_PI (M_PI * 2.0)
static VL53L1_Dev_t Device;
static uint16_t distance = 0;
static uint8_t tmp_alti = 0; // used to check if a new altimeter data is ready
static int16_t status = 0; // altimeter status
state_estimate_t state_estimate;  // extern variable in state_estimator.h

rc_ringbuf_t z_alti_track;  //help tracking the velocity of the plane without MOCAP

// sensor data structs
rc_mpu_data_t mpu_data;
rc_vector_t accel_in = RC_VECTOR_INITIALIZER;
rc_vector_t accel_out = RC_VECTOR_INITIALIZER;
rc_matrix_t rot_matrix = RC_MATRIX_INITIALIZER;
static rc_bmp_data_t bmp_data;

// gyro filter
static rc_filter_t gyro_roll_lpf = RC_FILTER_INITIALIZER;
static rc_filter_t gyro_pitch_lpf = RC_FILTER_INITIALIZER;
static rc_filter_t gyro_yaw_lpf = RC_FILTER_INITIALIZER;

// z filter
static rc_filter_t z_lpf = RC_FILTER_INITIALIZER;

// battery filter
static rc_filter_t batt_lp = RC_FILTER_INITIALIZER;

// altitude filter components
static rc_kalman_t alt_kf = RC_KALMAN_INITIALIZER;
static rc_kalman_t horiz_pos_estimator  = RC_KALMAN_INITIALIZER;
//static rc_filter_t acc_lp = RC_FILTER_INITIALIZER;
static rc_filter_t z_velocity_lp = RC_FILTER_INITIALIZER;

static rc_vector_t pos_u   = RC_VECTOR_INITIALIZER;
static rc_vector_t pos_y   = RC_VECTOR_INITIALIZER;

static void __batt_init(void)
{
    // init the battery low pass filter
    rc_filter_moving_average(&batt_lp, 20, DT);
    double tmp = rc_adc_dc_jack();
    if (tmp < 3.0)
    {
        tmp = settings.v_nominal;
        if (settings.warnings_en)
        {
            fprintf(stderr, "WARNING: ADC read %0.1fV on the barrel jack. Please connect\n", tmp);
            fprintf(stderr, "battery to barrel jack, assuming nominal voltage for now.\n");
        }
    }
    rc_filter_prefill_inputs(&batt_lp, tmp);
    rc_filter_prefill_outputs(&batt_lp, tmp);
    return;
}

static void __batt_march(void)
{
    double tmp = rc_adc_dc_jack();
    if (tmp < 3.0) tmp = settings.v_nominal;
    state_estimate.v_batt_raw = tmp;
    state_estimate.v_batt_lp = rc_filter_march(&batt_lp, tmp);
    return;
}

static void __batt_cleanup(void)
{
    rc_filter_free(&batt_lp);
    return;
}

static void __gyro_init(void)
{
    rc_filter_first_order_lowpass(&gyro_pitch_lpf, DT, 3*DT);
    rc_filter_first_order_lowpass(&gyro_roll_lpf, DT, 3*DT);
    rc_filter_first_order_lowpass(&gyro_yaw_lpf, DT, 3*DT);
    return;
}

static void __gyro_march(void)
{
    state_estimate.roll_dot = rc_filter_march(&gyro_roll_lpf, state_estimate.gyro[0]);
    state_estimate.pitch_dot = rc_filter_march(&gyro_pitch_lpf, state_estimate.gyro[1]);
    state_estimate.yaw_dot = rc_filter_march(&gyro_yaw_lpf, state_estimate.gyro[2]);
    return;
}

static void __gyro_cleanup(void)
{
    rc_filter_free(&gyro_pitch_lpf);
    rc_filter_free(&gyro_roll_lpf);
    rc_filter_free(&gyro_yaw_lpf);
    return;
}

static void __alti_init(void)
{
    printf("Initializing the alitimeter... \n");
	uint8_t addr = VL53L1X_DEFAULT_DEVICE_ADDRESS;
	uint8_t i2cbus = 1;
	uint16_t rtn;

    status = VL53L1X_InitDriver(&Device, i2cbus, addr);
	if(status!=0){
		printf("ERROR: VL53LX Not Responding\n");
	}

    VL53L1X_SensorInit(&Device);

    VL53L1X_GetDistanceMode(&Device,&rtn);

    printf("Altimeter Distance Mode: %d\n", rtn);

    VL53L1X_GetInterMeasurementInMs(&Device,&rtn);
	printf("Measurement Period: %dms\n", rtn);
	uint16_t rate = rtn;

	VL53L1X_GetTimingBudgetInMs(&Device,&rtn);
	printf("Timing Budget: %dms\n", rtn);

    VL53L1X_StartRanging(&Device); 
    return;
}

static void __z_init(void)
{
    rc_filter_first_order_lowpass(&z_lpf, DT, 300*DT);
    return;
}

static void __z_march(void)
{
    // state_estimate.Z_ddot = rc_filter_march(&z_lpf, state_estimate.accel_ground_frame[2]);
    state_estimate.Z_ddot = 0;
    return;
}

static void __z_cleanup(void)
{
    rc_filter_free(&z_lpf);
    return;
}

static void __imu_march(void)
{
    static double last_yaw = 0.0;
    static int num_yaw_spins = 0;
    double diff;

    // gyro and accel require converting to NED coordinates
    state_estimate.gyro[0] =  mpu_data.gyro[1] * DEG_TO_RAD;
    state_estimate.gyro[1] =  mpu_data.gyro[0] * DEG_TO_RAD;
    state_estimate.gyro[2] = -mpu_data.gyro[2] * DEG_TO_RAD;

    // quaternion also needs coordinate transform
    state_estimate.quat_imu[0] = mpu_data.dmp_quat[0];   // W
    state_estimate.quat_imu[1] = mpu_data.dmp_quat[2];   // X (i)
    state_estimate.quat_imu[2] = mpu_data.dmp_quat[1];   // Y (j)
    state_estimate.quat_imu[3] = -mpu_data.dmp_quat[3];  // Z (k)

    // normalize it just in case
    rc_quaternion_norm_array(state_estimate.quat_imu);
    // generate tait bryan angles
    rc_quaternion_to_tb_array(state_estimate.quat_imu, state_estimate.tb_imu);

    // yaw is more annoying since we have to detect spins
    // also make sign negative since NED coordinates has Z point down
    diff = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI) - last_yaw;
    // detect the crossover point at +-PI and update num yaw spins
    if (diff < -M_PI)
        num_yaw_spins++;
    else if (diff > M_PI)
        num_yaw_spins--;

    // finally the new value can be written
    state_estimate.imu_continuous_yaw = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI);
    last_yaw = state_estimate.imu_continuous_yaw;

    state_estimate.accel[0] = mpu_data.accel[1];
    state_estimate.accel[1] = mpu_data.accel[0];
    state_estimate.accel[2] = -mpu_data.accel[2];
    
    accel_in.d[0] = state_estimate.accel[0];
    accel_in.d[1] = state_estimate.accel[1];
    accel_in.d[2] = state_estimate.accel[2];

    // rotate accel vector
    rc_quaternion_rotate_vector_array(accel_in.d, state_estimate.quat_imu);

    state_estimate.accel_ground_frame[0] = accel_in.d[0];
    state_estimate.accel_ground_frame[1] = accel_in.d[1];
    state_estimate.accel_ground_frame[2] = accel_in.d[2] + GRAVITY;
    return;
}

static void __mag_march(void)
{
    static double last_yaw = 0.0;
    static int num_yaw_spins = 0;

    // don't do anything if mag isn't enabled
    if (!settings.enable_magnetometer) return;

    // mag require converting to NED coordinates
    state_estimate.mag[0] = mpu_data.mag[1];
    state_estimate.mag[1] = mpu_data.mag[0];
    state_estimate.mag[2] = -mpu_data.mag[2];

    // quaternion also needs coordinate transform
    state_estimate.quat_mag[0] = mpu_data.fused_quat[0];   // W
    state_estimate.quat_mag[1] = mpu_data.fused_quat[2];   // X (i)
    state_estimate.quat_mag[2] = mpu_data.fused_quat[1];   // Y (j)
    state_estimate.quat_mag[3] = -mpu_data.fused_quat[3];  // Z (k)

    // normalize it just in case
    rc_quaternion_norm_array(state_estimate.quat_mag);
    // generate tait bryan angles
    rc_quaternion_to_tb_array(state_estimate.quat_mag, state_estimate.tb_mag);

    // heading
    state_estimate.mag_heading_raw = mpu_data.compass_heading_raw;
    state_estimate.mag_heading = state_estimate.tb_mag[2];

    // yaw is more annoying since we have to detect spins
    // also make sign negative since NED coordinates has Z point down
    double diff = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI) - last_yaw;
    // detect the crossover point at +-PI and update num yaw spins
    if (diff < -M_PI)
        num_yaw_spins++;
    else if (diff > M_PI)
        num_yaw_spins--;

    // finally the new value can be written
    state_estimate.mag_heading_continuous = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI);
    last_yaw = state_estimate.mag_heading_continuous;
    return;
}

/**
 * @brief      initialize the altitude kalman filter
 *
 * @return     0 on success, -1 on failure
 */
static int __altitude_init(void)
{
    // initialize altitude kalman filter and bmp sensor
    rc_matrix_t F = RC_MATRIX_INITIALIZER;
    rc_matrix_t G = RC_MATRIX_INITIALIZER;
    rc_matrix_t H = RC_MATRIX_INITIALIZER;
    rc_matrix_t Q = RC_MATRIX_INITIALIZER;
    rc_matrix_t R = RC_MATRIX_INITIALIZER;
    rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

    const int Nx = 3;
    const int Ny = 1;
    const int Nu = 1;

    // allocate appropirate memory for system
    rc_matrix_zeros(&F, Nx, Nx);
    rc_matrix_zeros(&G, Nx, Nu);
    rc_matrix_zeros(&H, Ny, Nx);
    rc_matrix_zeros(&Q, Nx, Nx);
    rc_matrix_zeros(&R, Ny, Ny);
    rc_matrix_zeros(&Pi, Nx, Nx);

    // define system -DT; // accel bias
    F.d[0][0] = 1.0;
    F.d[0][1] = DT;
    F.d[0][2] = 0.0;
    F.d[1][0] = 0.0;
    F.d[1][1] = 1.0;
    F.d[1][2] = -DT;  // subtract accel bias
    F.d[2][0] = 0.0;
    F.d[2][1] = 0.0;
    F.d[2][2] = 1.0;  // accel bias state

    G.d[0][0] = -0.5 * DT * DT;
    G.d[0][1] = -DT;
    G.d[0][2] = 0.0;

    H.d[0][0] = 1.0;
    
    H.d[0][1] = 0.0;
    H.d[0][2] = 0.0;

    // covariance matrices
    Q.d[0][0] = 0;
    Q.d[1][1] = 0.1428; 
    Q.d[2][2] = 0.1;  // small value 
    R.d[0][0] = 0.25;

    //This works well in simulation
    Pi.d[0][0] = 1;
    Pi.d[1][1] = 1;
    Pi.d[2][2] = 1;

    // initialize the kalman filter
    if (rc_kalman_alloc_lin(&alt_kf, F, G, H, Q, R, Pi) == -1) return -1;
    rc_matrix_free(&F);
    rc_matrix_free(&G);
    rc_matrix_free(&H);
    rc_matrix_free(&Q);
    rc_matrix_free(&R);
    rc_matrix_free(&Pi);

    // init barometer and read in first data
    if (rc_bmp_read(&bmp_data)) return -1;

    //init lp filter for z velocity
    rc_filter_first_order_lowpass(&z_velocity_lp, DT, 3*DT);

    return 0;
}



static void __alitimeter_march(void)
{
    status = VL53L1X_CheckForDataReady(&Device, &tmp_alti);
			// rc_usleep(1E2);
    if(tmp_alti != 0){
        VL53L1X_ClearInterrupt(&Device);
        VL53L1X_GetDistance(&Device, &distance);
        state_estimate.alt_altimeter = -(double)distance /1000; // converted to meter
    }

}

static void __altitude_march(void)
{
    int i;
    double accel_vec[3];
    static rc_vector_t u = RC_VECTOR_INITIALIZER;
    static rc_vector_t y = RC_VECTOR_INITIALIZER;
    // static double global_update;

    // // grab raw data (not used if other global update used)
    // state_estimate.bmp_pressure_raw = bmp_data.pressure_pa;
    // state_estimate.alt_bmp_raw = bmp_data.alt_m;
    // state_estimate.bmp_temp = bmp_data.temp_c;

    // // Set Global update variable
    // global_update = gps_data.lla.alt;

    // make copy of acceleration reading before rotating
    for (i = 0; i < 3; i++) accel_vec[i] = state_estimate.accel[i];

    // rotate accel vector to ground frame (z up)
    rc_quaternion_rotate_vector_array(accel_vec, state_estimate.quat_imu);

    if (alt_kf.step == 0)
    {
        rc_vector_zeros(&u, 1);
        rc_vector_zeros(&y, 1);
    }

   // reverse the z acceleration to point down
    u.d[0] = -accel_vec[2] - GRAVITY;
    state_estimate.alt_accelometer = -accel_vec[2]- GRAVITY; //acceleration in z without gavity
    

    // Propagate altitude based on dynamic model x(k+1) = A*x(k) + B*u(k)
	kalman_predict(&alt_kf, u);

	// Just in case there are no new measurements available, use the a priori
	// estimate as the state estimate. kalman_correct() calls will update this.
	alt_kf.x_est.d[0] = alt_kf.x_pre.d[0];
	alt_kf.x_est.d[1] = alt_kf.x_pre.d[1];
 
    // If the range finder has a new measurement...
    // Update the altitute with the range finder
    status = VL53L1X_CheckForDataReady(&Device, &tmp_alti); // check if there is a new measurement
	if(tmp_alti){

		// Get the range finder's latest measurement.
		VL53L1X_GetDistance(&Device, &distance);

		// Convert it to meters and subtract the bias.
		state_estimate.alt_altimeter = -(double)distance/1000.0;

		// Project the measurement onto the inertial frame Z-axis
		state_estimate.alt_altimeter = state_estimate.alt_altimeter *
				cos(state_estimate.roll) * cos(state_estimate.pitch);

		// Set the measurement "y" to the range finder reading.
		y.d[0] = state_estimate.alt_altimeter;

		// Correct the state estimate
		kalman_correct(&alt_kf, y);

		// Turn off range finder measurement ready flag and clear the interrupt
		tmp_alti = 0;
		VL53L1X_ClearInterrupt(&Device);
	}

    // altitude estimate
    state_estimate.alt_estimate = alt_kf.x_est.d[0];
    //state_estimate.alt_velocity = alt_kf.x_est.d[1];
    // state_estimate.alt_bmp_accel = alt_kf.x_est.d[2];

    // Calculate alt_velocity
    rc_ringbuf_insert(&z_alti_track, state_estimate.alt_estimate);
    state_estimate.alt_velocity = __diff_function_helper(&z_alti_track, state_estimate.alt_velocity, DT);
    state_estimate.alt_velocity = rc_filter_march(&z_velocity_lp, state_estimate.alt_velocity);

    return;
}

/**
 * @brief   Select state estimate values based on flight mode
 */
static void __feedback_select(void)
{
    switch (user_input.flight_mode)
    {
        case AUTONOMOUS:
        case LOITER:
        case LOITER_RSP:
            state_estimate.roll = state_estimate.tb_imu[0];
            state_estimate.pitch = state_estimate.tb_imu[1];
            state_estimate.yaw = state_estimate.tb_imu[2];
            state_estimate.continuous_yaw =
                atan2(2 * (xbeeMsg.qw * xbeeMsg.qz + xbeeMsg.qx * xbeeMsg.qy),
                    1 - 2 * (pow(xbeeMsg.qy, 2) + pow(xbeeMsg.qz, 2)));
            state_estimate.X = xbeeMsg.x;  // TODO: generalize for optitrack and qualisys
            state_estimate.Y = xbeeMsg.y;
            state_estimate.Z = xbeeMsg.z;
            state_estimate.X_dot = xbee_x_dot;
            state_estimate.Y_dot = xbee_y_dot;
            state_estimate.Z_dot = xbee_z_dot;
            break;
            
        case TAKE_OFF:
        case LAND:
        case FOLLOW_ME:
            state_estimate.roll = state_estimate.tb_imu[0];
            state_estimate.pitch = state_estimate.tb_imu[1];
            state_estimate.yaw = state_estimate.tb_imu[2];
            
            if (socket_object_tracking()) 
            {
                // Reset state_estimate.delta_yaw = 0 everytime when we get new data from the socket (implemented in socket_manager.c)
                // Otherwise, 
                // 1) integrate the gyro data over time   
                state_estimate.delta_yaw += state_estimate.yaw_dot * DT;
                // 2) or use the visual odometry
                // state_estimate.delta_yaw = visual_odometry.roll - roll_reference; // need to reset the reference when it gets new data 

                state_estimate.object_tracking = true;
            }
            else 
            {
                state_estimate.object_tracking = false;
            }

            // This need mocap
            state_estimate.continuous_yaw =
                 atan2(2 * (xbeeMsg.qw * xbeeMsg.qz + xbeeMsg.qx * xbeeMsg.qy),
                     1 - 2 * (pow(xbeeMsg.qy, 2) + pow(xbeeMsg.qz, 2)));

            //state_estimate.continuous_yaw = state_estimate.mag_heading_continuous;
            state_estimate.X = xbeeMsg.x;  // TODO: generalize for optitrack and qualisys
            state_estimate.Y = xbeeMsg.y;
            // state_estimate.Z = xbeeMsg.z;

            state_estimate.Z = state_estimate.alt_estimate;  //use the kalman filtered reading
        

            state_estimate.X_dot = xbee_x_dot;
            state_estimate.Y_dot = xbee_y_dot;  
            // state_estimate.Z_dot = xbee_z_dot;
            state_estimate.Z_dot = state_estimate.alt_velocity;

            state_estimate.visual_range = object_observation.range;
            state_estimate.visual_bearing = object_observation.bearing;
            
            break;

        default:
            state_estimate.roll = state_estimate.tb_imu[0];
            state_estimate.pitch = state_estimate.tb_imu[1];
            state_estimate.yaw = state_estimate.tb_imu[2];
            state_estimate.continuous_yaw = state_estimate.mag_heading_continuous;
            state_estimate.X = xbeeMsg.x;
            state_estimate.Y = xbeeMsg.y;
            state_estimate.Z = xbeeMsg.z;
            state_estimate.X_dot = xbee_x_dot;
            state_estimate.Y_dot = xbee_y_dot;
            state_estimate.Z_dot = xbee_z_dot;
            break;
    }
}

static void __filter_cleanup(void)
{
    rc_kalman_free(&alt_kf);
    //rc_filter_free(&acc_lp);
    rc_vector_free(&pos_u);
	rc_vector_free(&pos_y);
    rc_filter_free(&z_velocity_lp);

    // if using x y filter 
    //rc_kalman_free(&horiz_pos_estimator);
    return;
}

static void __mocap_check_timeout(void)
{
    if (state_estimate.mocap_running)
    {
        uint64_t current_time = rc_nanos_since_epoch();
        // check if mocap data is > 3 steps old
        if ((current_time - state_estimate.mocap_timestamp_ns) > (3 * 1E7))
        {
            state_estimate.mocap_running = 0;
            if (settings.warnings_en)
            {
                fprintf(stderr, "WARNING, MOCAP LOST VISUAL\n");
            }
        }
    }
    return;
}

// object tracking check timeout


int state_estimator_init(void)
{
    __batt_init();
    __gyro_init();
    __alti_init();
    //__z_init();
    if (__altitude_init() == -1) return -1;
    if (rc_vector_zeros(&accel_in, 3) == -1) return -1;
    if (rc_vector_zeros(&accel_out, 3) == -1) return -1;
    if (rc_matrix_zeros(&rot_matrix, 3, 3) == -1) return -1;

    z_alti_track = rc_ringbuf_empty();
    if (rc_ringbuf_alloc(&z_alti_track, DIFF_POINTS) == -1) {
        printf("Failed to allocate z_position_track ringbuf");
        return -1;
    }
    //rc_ringbuf_insert(&z_alti_track, 0.0);

    state_estimate.initialized = 1;
    return 0;
}

int state_estimator_march(void)
{
    if (state_estimate.initialized == 0)
    {
        fprintf(stderr, "ERROR in state_estimator_march, estimator not initialized\n");
        return -1;
    }

    // populate state_estimate struct one setion at a time, top to bottom
    __batt_march();
    __imu_march();
    __mag_march();
    //__alitimeter_march(); the altimeter is update in __altitude_march()
    __altitude_march();
    __gyro_march();
    //estimate_horizontal_position();
    //__z_march();
    __feedback_select();
    __mocap_check_timeout();
    return 0;
}

int state_estimator_jobs_after_feedback(void)
{
    static int bmp_sample_counter = 0;

    // check if we need to sample BMP this loop
    if (bmp_sample_counter >= BMP_RATE_DIV)
    {
        // perform the i2c reads to the sensor, on bad read just try later
        if (rc_bmp_read(&bmp_data)) return -1;
        bmp_sample_counter = 0;
        state_estimate.bmp_time_ns = rc_nanos_since_epoch();
    }
    bmp_sample_counter++;
    return 0;
}

int state_estimator_cleanup(void)
{
    __batt_cleanup();
    __filter_cleanup();
    __gyro_cleanup();
    __z_cleanup();
    VL53L1X_StopRanging(&Device);
    return 0;
}


int kalman_predict(rc_kalman_t* kf, rc_vector_t u)
{
	/* Performs a Kalman filter time update to get "a priori" estimate. */

	rc_matrix_t newP = RC_MATRIX_INITIALIZER;
	rc_matrix_t FT = RC_MATRIX_INITIALIZER;
	rc_vector_t tmp1 = RC_VECTOR_INITIALIZER;
	rc_vector_t tmp2 = RC_VECTOR_INITIALIZER;

	// Sanity checks
	// if(unlikely(kf==NULL)){
	// 	fprintf(stderr, "ERROR in rc_kalman_lin_update, received NULL pointer\n");
	// 	return -1;
	// }
	// if(unlikely(kf->initialized !=1)){
	// 	fprintf(stderr, "ERROR in rc_kalman_lin_update, kf uninitialized\n");
	// 	return -1;
	// }
	// if(unlikely(u.initialized!=1)){
	// 	fprintf(stderr, "ERROR in rc_kalman_lin_update received uninitialized vector\n");
	// 	return -1;
	// }
	// if(unlikely(u.len != kf->G.cols)){
	// 	fprintf(stderr, "ERROR in rc_kalman_lin_update u must have same dimension as columns of G\n");
	// 	return -1;
	// }


	// for linear case only, calculate x_pre from linear system model
	// x_pre = x[k|k-1] = F*x[k-1|k-1] +  G*u[k-1]
	rc_matrix_times_col_vec(kf->F, kf->x_est, &tmp1);
	rc_matrix_times_col_vec(kf->G, u, &tmp2);
	rc_vector_sum(tmp1, tmp2, &kf->x_pre);


	// Project the error covariance ahead
	// P[k|k-1] = F*P[k-1|k-1]*F^T + Q
	rc_matrix_multiply(kf->F, kf->P, &newP);	// newP = F*P_old
	rc_matrix_transpose(kf->F, &FT);
	rc_matrix_right_multiply_inplace(&newP, FT);	// P = F*P*F^T
	rc_matrix_add_inplace(&newP, kf->Q);		// P = F*P*F^T + Q
	rc_matrix_symmetrize(&newP);			// Force symmetric P
	rc_matrix_duplicate(newP,&kf->P);


	// cleanup
	rc_matrix_free(&newP);
	rc_matrix_free(&FT);
	rc_vector_free(&tmp1);
	rc_vector_free(&tmp2);

	kf->step++;
	return 0;
}

int kalman_correct(rc_kalman_t* kf, rc_vector_t y)
{
	/* Performs a Kalman measurement update to get "a posteriori" estimate. */

	rc_matrix_t L = RC_MATRIX_INITIALIZER;
	rc_matrix_t newP = RC_MATRIX_INITIALIZER;
	rc_matrix_t S = RC_MATRIX_INITIALIZER;
	rc_vector_t h = RC_VECTOR_INITIALIZER;
	rc_vector_t z = RC_VECTOR_INITIALIZER;
	rc_vector_t tmp1 = RC_VECTOR_INITIALIZER;

	// Sanity checks
	// if(unlikely(y.initialized!=1)){
	// 	fprintf(stderr, "ERROR in rc_kalman_lin_update received uninitialized vector\n");
	// 	return -1;
	// }
	// if(unlikely(y.len != kf->H.rows)){
	// 	fprintf(stderr, "ERROR in rc_kalman_lin_update y must have same dimension as rows of H\n");
	// 	return -1;
	// }

	// Get the output based on the a priori state estimate
	// h[k] = H * x_pre[k]
	rc_matrix_times_col_vec(kf->H,kf->x_pre,&h);


	// Calculate the Innovation Covariance
	// S = H*P*H^T + R
	// Calculate H^T, borrow S for H^T
	rc_matrix_transpose(kf->H, &S);			// S = H^T
	// Calculate a part of L in advance before we modify S = H^T
	rc_matrix_duplicate(kf->P, &newP);
	rc_matrix_multiply(newP, S, &L);		// K = P*(H^T)
	rc_matrix_left_multiply_inplace(newP, &S);	// S = P*H^T
	rc_matrix_left_multiply_inplace(kf->H, &S);	// S = H*(P*H^T)
	rc_matrix_add_inplace(&S, kf->R);		// S = H*P*H^T + R


	// Calculate the Kalman Gain
	// L = P*(H^T)*(S^-1)
	//rc_matrix_print(S);
	rc_algebra_invert_matrix_inplace(&S);		// S2^(-1) = S^(-1)
	rc_matrix_right_multiply_inplace(&L, S);	// L = (P*H^T)*(S^-1)


	// Calculate Innovation
	// x[k|k] = x[k|k-1] + K[k]*(y[k]-h[k])
	rc_vector_subtract(y,h,&z);			        // z = k-h


	// Correct the state estimate (a posteriori)
	rc_matrix_times_col_vec(L, z, &tmp1);		// temp = L*z
	rc_vector_sum(kf->x_pre, tmp1, &kf->x_est);	// x_est = x + K*y


	// Correct the error covariance (a posteriori)
	// P[k|k] = (I - L*H)*P = P[k|k-1] - L*H*P[k|k-1], reuse the matrix S.
	rc_matrix_multiply(kf->H, newP, &S);		// S = H*P
	rc_matrix_left_multiply_inplace(L, &S);		// S = L*(H*P)
	rc_matrix_subtract_inplace(&newP, S);		// P = P - L*H*P
	rc_matrix_symmetrize(&newP);			// Force symmetric P
	rc_matrix_duplicate(newP,&kf->P);


	// cleanup
	rc_matrix_free(&L);
	rc_matrix_free(&newP);
	rc_matrix_free(&S);
	rc_vector_free(&h);
	rc_vector_free(&z);
	rc_vector_free(&tmp1);

	kf->step++;
	return 0;
}

int initialize_horizontal_position_estimator(void)
{
	/* Initializes a Kalman filter and the Xbee radio. */

	const int Nx = 6;
	const int Ny = 2;
	const int Nu = 2;

	// Initialize Kalman filter matrices
	rc_matrix_t A   = RC_MATRIX_INITIALIZER;
	rc_matrix_t B   = RC_MATRIX_INITIALIZER;
	rc_matrix_t C   = RC_MATRIX_INITIALIZER;
	rc_matrix_t W   = RC_MATRIX_INITIALIZER;
	rc_matrix_t V   = RC_MATRIX_INITIALIZER;
	rc_matrix_t P   = RC_MATRIX_INITIALIZER;


	// Allocate appropriate memory for system
	rc_matrix_zeros(&A, Nx, Nx);
	rc_matrix_zeros(&B, Nx, Nu);
	rc_matrix_zeros(&C, Ny, Nx);
	rc_matrix_zeros(&W, Nx, Nx);
	rc_matrix_zeros(&V, Ny, Ny);
	rc_matrix_zeros(&P, Nx, Nx);
	rc_vector_zeros(&pos_u, Nu);
	rc_vector_zeros(&pos_y, Ny);


	// Discrete-Time Plant State Space Model
	// State Vector: x, y, vel_x, vel_y, acc_x_bias, accel_y_bias
	A.d[0][0] = 1.0;
	A.d[0][2] = DT;
	A.d[1][1] = 1.0;
	A.d[1][3] = DT;
	A.d[2][2] = 1.0;
	A.d[2][4] = -DT;
	A.d[3][3] = 1.0;
	A.d[3][5] = -DT;
	A.d[4][4] = 1.0;
	A.d[5][5] = 1.0;

	B.d[0][0] = -0.00001177;
	B.d[2][0] = -0.00470810;
	B.d[1][1] = -0.00001177;
	B.d[3][1] = -0.00470810;

	C.d[0][0] = 1.0;
	C.d[1][1] = 1.0;


	// Process Noise Covariance Matrix
	W.d[2][2] = 0.1428;
	W.d[3][3] = 0.1428;
	W.d[4][4] = 0.1; // Smaller value makes cov changes more slowly
	W.d[5][5] = 0.1; // Smaller value makes cov changes more slowly


	// Sensor Noise Covariance Matrix
	V.d[0][0] = 0.1;
	V.d[1][1] = 0.1;


	// Initial Error Covariance Matrix
	P.d[0][0] = 1.0;
	P.d[1][1] = 1.0;
	P.d[2][2] = 1.0;
	P.d[3][3] = 1.0;
	P.d[4][4] = 1.0;
	P.d[5][5] = 1.0;


	if(rc_kalman_alloc_lin(&horiz_pos_estimator,A,B,C,W,V,P)==-1) {
		printf("rc_kalman_alloc_lin failed\n");
		return -1;
	}

	// Free up the memory
	rc_matrix_free(&A);
	rc_matrix_free(&B);
	rc_matrix_free(&C);
	rc_matrix_free(&W);
	rc_matrix_free(&V);
	rc_matrix_free(&P);


	return 0;
}

static void estimate_horizontal_position(void)
{
	/* Estimates quadrotor horizontal position (in meters) via a linear Kalman
	 * filter. Fuses accelerometer and motion capture measurements. */


	// ========== KALMAN FILTER PREDICTION: Get "a priori" estimate ==========

	// Set process measurement "u" to acceleration along x and y axes.
	pos_u.d[0] = state_estimate.accel_ground_frame[0];
	pos_u.d[1] = state_estimate.accel_ground_frame[0];

	// Propagate altitude based on dynamic model x(k+1) = A*x(k) + B*u(k)
	kalman_predict(&horiz_pos_estimator, pos_u);

	// Just in case there are no new measurements available, use the a priori
	// estimate as the state estimate. kalman_correct() calls will update this.
	horiz_pos_estimator.x_est.d[0] = horiz_pos_estimator.x_pre.d[0];
	horiz_pos_estimator.x_est.d[1] = horiz_pos_estimator.x_pre.d[1];



	// ===== KALMAN FILTER CORRECTION: Sequential Measurement Processing =====


	// If a new data from the phone is ready
	// if(settings.xbee_en && xbee_data_ready){


	// 	// Set the measurement "y" to the range finder reading.
	// 	pos_y.d[0] = xbeeMsg.x;
	// 	pos_y.d[1] = xbeeMsg.y;

	// 	// Correct the state estimate
	// 	kalman_correct(&horiz_pos_estimator, pos_y);
    // }


	// Set the new horizontal position estimate
	state_estimate.X_filter = horiz_pos_estimator.x_est.d[0];
	state_estimate.Y_filter = horiz_pos_estimator.x_est.d[1];
	state_estimate.X_dot_filter = horiz_pos_estimator.x_est.d[2];
	state_estimate.Y_dot_filter = horiz_pos_estimator.x_est.d[3];

	return;
}



