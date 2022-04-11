/**
 * <printf_manager.h>
 *
 * @brief   Functions to start and stop the printf mnaager which is a
 * separate thread printing data to the console for debugging.
 *
 * @addtogroup PrintfManager
 * @{
 */

#ifndef __SOCKET_MANAGER__
#define __SOCKET_MANAGER__

#include <flight_mode.h>
#include <stdint.h>

// typedef struct pose_xyt_t
// {    
//     double x;
//     double y;
//     double theta;
// } pose_xyt_t;

typedef struct message_type_t 
{
    uint64_t msg_type;
} message_type_t;

typedef struct object_observation_t 
{
    uint64_t msg_type;
    volatile double bearing;
    volatile double position_y;
    volatile double range;
} object_observation_t;

typedef struct visual_odometry_t
{
    uint64_t msg_type;
    volatile double x;
    volatile double y;
    volatile double z;
    volatile double roll;
    volatile double pitch;
    volatile double yaw;
} visual_odometry_t;

// static pthread_t socket_manager_thread;
typedef struct thread_info
{
    int num;
    pthread_t socket_manager_thread;

    object_observation_t* obj_obsrv_buffer; 
    visual_odometry_t* visual_od_buf;
    
    int new_socket;
    int valread;
    char buffer[1024];

    volatile uint64_t socket_last_received_time_ns; 
    volatile uint64_t obj_obsrv_last_received_time_ns;
    volatile uint64_t visual_od_last_received_time_ns;
} thread_info_t;

extern object_observation_t object_observation;
extern visual_odometry_t visual_odometry;
thread_info_t server_threadinfo;


/**
 * @brief   Start the printf_manager thread which should be the only thing
 * printing to the screen besides error messages from other threads.
 *
 * @return  0 on success, -1 on failure
 */
int socketserver_init(void);

/**
 * @brief   Waits for the printf manager thread to exit.
 *
 * @return  0 on clean exit, -1 on exit time out/force close
 */
int socketserver_cleanup(void);


#endif /* __SOCKET_MANAGER__ */

/* @} end group PrintfManager */
