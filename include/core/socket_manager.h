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

typedef struct pose_xyt_t
{    
    double x;
    double y;
    double theta;
}pose_xyt_t;

// static pthread_t socket_manager_thread;
typedef struct thread_info
{
    int num;
    pthread_t socket_manager_thread;
    pose_xyt_t* tempbuf;
    Image_data_t* image_data_buff; // Image data type
    int new_socket;
    int valread;
    char buffer[1024];
    uint64_t socket_last_received_time_ns;
} thread_info_t;

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
