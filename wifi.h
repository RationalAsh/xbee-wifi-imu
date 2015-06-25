#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#define BUFFER_SIZE 30    												 // Increase this as required

int init_wifi(int bot_port,char* bot_address);
void close_socket(int bot1_sockfd);
int establishConnection(char* bot_address, int bot_portno);
char* tcpRead( int socket_descriptor, char *buffer);
int tcpWrite(int socket_descriptor,char* message);
void error(const char *msg);

#ifdef __cplusplus
}
#endif
