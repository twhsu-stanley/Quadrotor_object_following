// Server side C/C++ program to demonstrate Socket programming
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#define PORT 8080
typedef struct pose_xyt_t
{    
    double x;
    double y;
    double theta;
}pose_xyt_t;

pose_xyt_t* tempbuf;
void *do_stuff(void *user);


typedef struct thread_info {
    int num;
    pthread_t thread;
    int new_socket;
    int valread;
    char  buffer[1024];
} thread_info_t;

void socketsetup() {
	printf("entered socketsetup\n");
	fflush(stdout);
    int server_fd, new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
//	char buffer[1024] = {0};
	char *hello = "Hello from server";

    thread_info_t server_threadinfo;
    server_threadinfo.num = 1;


    	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}
	
	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
								&opt, sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons( PORT );
	
	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr *)&address,
								sizeof(address))<0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	if (listen(server_fd, 3) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}
	if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
					(socklen_t*)&addrlen))<0)
	{
		perror("accept");
		exit(EXIT_FAILURE);
	}

    server_threadinfo.new_socket=new_socket;
    server_threadinfo.valread = valread;
  //  server_threadinfo.buffer = (char **)buffer;
   //server_threadinfo.thread 
    printf("about to make the thread\n");
    fflush(stdout);
    pthread_create(&server_threadinfo.thread, NULL, do_stuff, &server_threadinfo);
    pthread_join(server_threadinfo.thread, NULL);
}


int main(int argc, char const *argv[])
{

//	pthread_t serverthread;

    socketsetup();
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  printf("now: %d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	return 0;
}


// bool producer_finished = false;

void *do_stuff(void *user) {
    // if (producer_finished) {
    //     exit(0);
    // }
    // do stuff
  //buffer[1024] = {0};
//	printf("entered the thread");
	fflush(stdout);
        thread_info_t *info = user;

//	info->buffer = {0};
	while(1) {
//	printf("casted the pointer");
	fflush(stdout);
        info->valread = read( info->new_socket , info->buffer, 1024);
//	printf("assigning the read valuez");
//	fflush(stdout);
//        printf("%s\n",info->buffer);
//	fflush(stdout);
        tempbuf = (pose_xyt_t*)&info->buffer;
        printf("X: %f\tY: %f\tDepth: %f\n",tempbuf->x,tempbuf->y,tempbuf->theta);
fflush(stdout);      	
send(info->new_socket , "hello from server" , 17 , 0 );
}
}

// int main(void) {
//     // start another thread that changes producer_finished
//     while (true) {
//         do_stuff();
//     }
// }