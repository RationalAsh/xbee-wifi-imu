#include <stdio.h>
#include <math.h>
#include "wifi.c"


//For XBEE-1
#define XB0IP "192.168.1.150"
#define XB1IP "192.168.1.151"
#define XB2IP "192.168.1.152"
#define XB3IP "192.168.1.153"
#define XB4IP "192.168.1.154"
#define XB5IP "192.168.1.155"
#define XB6IP "192.168.1.156"
#define LOCAL "127.0.0.1"
#define TESTPORT 5000
#define XBPORT 9750
#define MSGBUFSIZE 1024
#define DOF 3
#define SENSORS 7
#define TIMEOUT 1 //Seconds

char msg1[MSGBUFSIZE];
char msg2[MSGBUFSIZE];
char reply[MSGBUFSIZE];
char reply1[MSGBUFSIZE];
char reply2[MSGBUFSIZE];
double sensor_readings[SENSORS][DOF];
double quaternions[SENSORS][4];
int sensormask[SENSORS] = {0,1,1,0,0,0,0};

//Structs for IO waiting
fd_set master;
fd_set keypress;

void read_data(int *sockets);
void update_ypr(char *data_str, int sensor_number);
void print_readings();
void print_quaternions();
void update_quaternion(int sensor_number);
void connect_xbees();

int main(int argc, char** argv)
{
    int i, j;
    int xbsocks[SENSORS];
    char **xbips;

    //Array of IPS
    xbips = (char**)malloc(SENSORS*sizeof(char*));
    for(i=0;i<SENSORS;i++)
	xbips[i] = (char *)malloc(16*sizeof(char));

    xbips[0] = XB0IP;
    xbips[1] = XB1IP;
    xbips[2] = XB2IP;
    xbips[3] = XB3IP;
    xbips[4] = XB4IP;
    xbips[5] = XB5IP;
    xbips[6] = XB6IP;	    
    
    //Value for the timeout
    struct timeval timeout;
    timeout.tv_sec = TIMEOUT;
    timeout.tv_usec = 0;

    //Connect to all the xbees
    for(i=0;i<SENSORS;i++)
    {
	if(sensormask[i])
	    xbsocks[i] = establishConnection(xbips[i], XBPORT);
    }

    //Set up stdin with select
    int fd_stdin = fileno(stdin);

    //Clearing the sensor readings    
    for(i=0;i<SENSORS;i++)
	for(j=0;j<DOF;j++)
	    sensor_readings[i][j] = 0;

    //Send the start character to all the xbees
    for(i=0;i<SENSORS;i++)
    {
	if(sensormask[i])
	    tcpWrite(xbsocks[i], "aaa");
    }
    

    while(1)
    {
	//Setting up struct for TCP
	FD_ZERO(&master);
	for(i=0;i<SENSORS;i++)
	{
	    if(sensormask[i])
		FD_SET(xbsocks[i], &master);
	}

	//Wait for incoming data to be available
	int num_ready = select(FD_SETSIZE, &master, NULL, NULL, &timeout);

	//If data is received before timeout
	if(num_ready > 0)
	{
	    //Reads the data, converts it to quaternions and puts it 
	    //in the quaternions[SENSORS][4] array.
	    read_data(xbsocks);
	    //print_readings();
	    print_quaternions();
	}
    }


    return 0;
}


//Another helper function
void update_quaternion(int sensor_number)
{
    double *ypr = sensor_readings[sensor_number];
    
    double c1 = cos(ypr[0]/2.0);
    double c2 = cos(ypr[1]/2.0);
    double c3 = cos(ypr[2]/2.0);
    double s1 = sin(ypr[0]/2.0);
    double s2 = sin(ypr[1]/2.0);
    double s3 = sin(ypr[2]/2.0);

    double w = c1*c2*c3 - s1*s2*s3;
    double x = s1*s2*c3 + c1*c2*s3;
    double y = s1*c2*c3 + c1*s2*s3;
    double z = c1*s2*c3 - s1*c2*s3;

    quaternions[sensor_number][0] = w;
    quaternions[sensor_number][1] = x;
    quaternions[sensor_number][2] = y;
    quaternions[sensor_number][3] = z;    
}

//Helper function
void print_readings()
{
    int i, j;
    for(i=0;i<SENSORS;i++)
    {
	for(j=0;j<DOF;j++)
	{
	    printf("%.2f ", sensor_readings[i][j]);
	}
	printf("\n");
    }
}

void print_quaternions()
{
    int i, j;

    printf("w x y z\n");

    for(i=0;i<SENSORS;i++)
    {
	for(j=0;j<DOF+1;j++)
	{
	    printf("%.2f ", quaternions[i][j]);
	}
	printf("\n");
    }
}

//Helper function
void update_ypr(char *data_str, int sensor_number)
{
    char *token;

    int i;

    i = 0;
    while ((token = strsep(&data_str, ",")) != NULL)
    {
	double temp = atof(token);
	sensor_readings[sensor_number][i] = temp;
	//printf("%s\n", token);
	i++;
	//if(i>=3)
	//    break;
    }
}

void read_data(int *sockets)
{
    int i = 0;

    for(i=0; i<SENSORS; i++)
    {
	if(sensormask[i]){
	if((FD_ISSET(sockets[i], &master)))
	{
	    tcpRead(sockets[i], reply);
	    printf("Xbee%d: %s\n", i, reply);
	    char *temp = strdup(reply);
	    update_ypr(temp, i);
	    update_quaternion(i);
	}}
    }
}
