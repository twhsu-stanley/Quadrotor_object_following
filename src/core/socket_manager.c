/**
 * @file input_manager.c
 */

#include <errno.h>
#include <flight_mode.h>
#include <input_manager.h>
#include <math.h>  // for fabs
#include <rc/dsm.h>
#include <rc/math/other.h>
#include <rc/pthread.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc_pilot_defs.h>
#include <settings.h>
#include <state_estimator.h>
#include <stdio.h>
#include <thread_defs.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <socket_manager.h>
#include <rc_pilot_defs.h>
#include <image_receive.h>

#define PORT 8080

static bool obj_tracking = false;
static bool initialized = false;
void *__socket_manager_func(void *user);


int socketserver_init()
{
    if (settings.enable_socket)
    {
        printf("entered socketsetup\n");
        fflush(stdout);
        struct sockaddr_in address;
        int server_fd;
        int opt = 1;
        int addrlen = sizeof(address);
        //	char buffer[1024] = {0};
        // char *hello = "Hello from server";

        // thread_info_t server_threadinfo;
        server_threadinfo.num = 1;
//	server_threadinfo.buffer = {0};
        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            return -1;
        }

        // Forcefully attaching socket to the port 8080
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
        {
            perror("setsockopt");
            return -1;
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(PORT);

        // Forcefully attaching socket to the port 8080
        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            perror("bind failed");
            return -1;
        }
        if (listen(server_fd, 3) < 0)
        {
            perror("listen");
            return -1;
        }
        if ((server_threadinfo.new_socket =
                    accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
        {
            perror("accept");
            return -1;
        }
        //  server_threadinfo.buffer = (char **)buffer;
        // server_threadinfo.thread
printf("Thread info new socket %d\n",server_threadinfo.new_socket);
        printf("about to make the thread\n");
        fflush(stdout);
        initialized = true;
//        if (rc_pthread_create(&server_threadinfo.socket_manager_thread, &__socket_manager_func, NULL,
  //              SCHED_FIFO, OBJECT_DETECTION_HZ) == -1)
        if (rc_pthread_create(&server_threadinfo.socket_manager_thread, &__socket_manager_func, (void*) &server_threadinfo ,
                SCHED_FIFO, OBJECT_DETECTION_HZ) == -1)    
    {
            fprintf(stderr, "ERROR in start_socket_manager, failed to start thread\n");
            return -1;
        }
        rc_usleep(50000);
        return 0;
    }
    return 0;
}

int socketserver_cleanup()
{
    if ( mode_needs_socket(user_input.flight_mode) )
    // if(true)
    {
        int ret = 0;
        if (initialized)
        {
            // wait for the thread to exit
            ret = rc_pthread_timed_join(server_threadinfo.socket_manager_thread, NULL, PRINTF_MANAGER_TOUT);
            if (ret == 1)
                fprintf(stderr, "WARNING: printf_manager_thread exit timeout\n");
            else if (ret == -1)
                fprintf(stderr, "ERROR: failed to join printf_manager thread\n");
        }
        initialized = 0;
        return ret;
    }
}

void *__socket_manager_func(void *user)
{
    // if (producer_finished) {
    //     exit(0);
    // }
    // do stuff
    // buffer[1024] = {0};
    printf("entered the thread\n");
    fflush(stdout);
    thread_info_t *info = (thread_info_t *)user;

    //	info->buffer = {0};
    while (rc_get_state() != EXITING)
        {
            printf("enter the while loop\n");
            fflush(stdout);
//	printf("%d\t%x",info->new_socket,&info->new_socket);
    printf("Thread info new socket %d\n",info->new_socket);
    fflush(stdout);
            info->valread = read(info->new_socket,(void *) &info->buffer, 1024);
            printf("assigning the read valuez\n");
            fflush(stdout);
            // printf("%s\n",info->buffer);
            // fflush(stdout);
            if (info->valread > -1)
            {
            info->tempbuf = (pose_xyt_t *)&info->buffer;
            printf("X: %f\tY: %f\tDepth: %f\n", info->tempbuf->x, info->tempbuf->y, info->tempbuf->theta);
            
            // Image data type ///////////////////////////////////////////////////////////////////////////
            info->image_data_buff = (Image_data_t *)&info->buffer;
            printf("u: %f\tv: %f\trange: %f\tbearing: %f\n", info->image_data_buff->u, info->image_data_buff->v, info->image_data_buff->range, info->image_data_buff->bearing);
            //////////////////////////////////////////////////////////////////////////////////////////////

            fflush(stdout);
            send(info->new_socket, "hello from server", 17, 0);

            Image_data.bearing = info->tempbuf->theta;
            Image_data.u = info->tempbuf->x; 
            Image_data.v = info->tempbuf->y;
            Image_data.range = info->tempbuf->theta;

            info->socket_last_received_time_ns = rc_nanos_since_epoch();

            // Reset the referenced delta_yaw everytime when new socket data are obtained
            state_estimate.delta_yaw = 0;
            }
        }
        return NULL;
}