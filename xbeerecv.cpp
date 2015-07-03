#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "wifilib.h"

//Color codes for the terminal
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

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
#define TIMEOUT 2 //Seconds
#define ID_OFFSET 2
#define BASE_IP "192.168.253."


double sensor_readings[SENSORS][DOF];
double quaternions[SENSORS][4];
int sensormask[SENSORS] = {0,0,0,0,0,0,0};

//Structs for IO waiting
fd_set master;

int find_sensors(char* base_ip, int start, int end, int howmany, int xbsocks[SENSORS]);
void send_start_signal(int xbsocks[SENSORS], char *c);
int read_data(int xbsocks[SENSORS], double readings[SENSORS][DOF]);
void update_ypr(char *data_str, int sensor_number, double readings[SENSORS][DOF]);
void print_readings(double readings[SENSORS][DOF]);
void print_quaternions(double quats[SENSORS][4]);
void update_quaternion(double readings[SENSORS][DOF], 
		       int sensor_number, double quats[SENSORS][4]);
void calc_quaternions(double readings[SENSORS][DOF], double quats[SENSORS][4]);

int main(int argc, char** argv)
{
    int xbsocks[SENSORS];
    int i;

    //Clearing the socket array
    for(i=0;i<SENSORS;i++)
	xbsocks[i] = -1;

    /*
      Scan all the IP addresses from BASE_IP
      Find and connect to the sensors and store
      the scoket descriptors in an array
    */
    int num_found = find_sensors("172.23.38.",170, 255, 2, xbsocks);
    //int num_found = find_sensors("192.168.253.",100, 255, 2, xbsocks);

    /*
      Now that the connections are established to 
      all the detected Xbees, send a start character
      to each of them so that they start transmitting
      the readings
    */
    send_start_signal(xbsocks, "aaa");

    /*
      Now the xbees that are online are transmitting the
      yaw, pitch and roll readings. Read this data and 
      store them in an array and keep looping.
    */
    while(1)
    {
	//Read data into sensor_readings array
	int num = read_data(xbsocks, sensor_readings);

	//If there's any data
	if(num)
	{
	    print_readings(sensor_readings);
	    //Convert to quaternions and store in quaternion array
	    calc_quaternions(sensor_readings, quaternions);
	    print_quaternions(quaternions);
	}
    }
    

    return 0;
}

void calc_quaternions(double readings[SENSORS][DOF], double quats[SENSORS][4])
{
    int i;

    for(i=0;i<SENSORS;i++)
    {
	update_quaternion(readings, i, quats);
    }
} 

//A helper function
void update_quaternion(double readings[SENSORS][DOF], 
		       int sensor_number, double quats[SENSORS][4])
{
    double *ypr = readings[sensor_number];
    
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

    quats[sensor_number][0] = w;
    quats[sensor_number][1] = x;
    quats[sensor_number][2] = y;
    quats[sensor_number][3] = z;    
}

void print_quaternions(double quats[SENSORS][4])
{
    int i, j;

    printf("w x y z\n");

    for(i=0;i<SENSORS;i++)
    {
	for(j=0;j<4;j++)
	{
	    printf("%.2f ", quats[i][j]);
	}
	printf("\n");
    }
}

void print_readings(double readings[SENSORS][DOF])
{
    int i, j;
    for(i=0;i<SENSORS;i++)
    {
	for(j=0;j<DOF;j++)
	{
	    printf("%.2f ", readings[i][j]);
	}
	printf("\n");
    }
}

void update_ypr(char *data_str, int sensor_number, double readings[SENSORS][DOF])
{
    char *token;

    int i;

    i = 0;
    while ((token = strsep(&data_str, ",")) != NULL)
    {
	double temp = atof(token);
	readings[sensor_number][i] = temp;
	//printf("%s\n", token);
	i++;
	//if(i>=3)
	//    break;
    }
}

int read_data(int xbsocks[SENSORS], double readings[SENSORS][DOF])
{
    int i;
    char reply[MSGBUFSIZE];

    //Value for the timeout for reading
    struct timeval timeout;
    timeout.tv_sec = TIMEOUT;
    timeout.tv_usec = 0;

    FD_ZERO(&master);
    for(i=0;i<SENSORS;i++)
    {
	if(xbsocks[i] != -1)
	{
	    FD_SET(xbsocks[i], &master);
	}
    }

    int num_ready = select(FD_SETSIZE, &master, NULL, NULL, &timeout);
    if(num_ready > 0)
    {
	for(i=0;i<SENSORS;i++)
	{
	    if(xbsocks[i] != -1)
	    {
		if(FD_ISSET(xbsocks[i], &master))
		{
		    tcpRead(xbsocks[i], reply);
		    printf("Xbee%d: %s\n", i, reply);
		    char *temp = strdup(reply);
		    update_ypr(temp, i, readings);
		}
	    }
	}
    }

    return num_ready;
}
	    
	    

void send_start_signal(int xbsocks[SENSORS], char *c)
{
    int i;
    
    for(i=0;i<SENSORS;i++)
    {
	if(xbsocks[i] != -1)
	{
	    tcpWrite(xbsocks[i], c);
	}
    }
}

int find_sensors(char* base_ip, int start, int end, int howmany, int xbsocks[SENSORS])
{
    int i;
    char tmp[5];
    char tmpip[16];
    char reply[32];
    fd_set readlist;
    int num_detected = 0;
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 100000;

    for(i=start;i<=end;i++)
    {
	sprintf(tmp, "%d", i);
	sprintf(tmpip, "%s%s", base_ip, tmp);
	printf("Trying IP: %s\n", tmpip);
	int tmpfd = establishConnection(tmpip, XBPORT);
	if(tmpfd != 1) //Connected
	{
	    printf("Connection..\n");
	    FD_ZERO(&readlist);
	    FD_SET(tmpfd, &readlist);

	    tcpWrite(tmpfd, "???");
	    //Wait for 2 seconds for incoming data
	    select(FD_SETSIZE, &readlist, NULL, NULL, &timeout);
	    
	    tcpRead(tmpfd, reply);
	    printf("Got: %s", reply);

	    //if((reply[0] == 'x'))//&&(reply[1] == 'b'))
	    {
		printf("Found and xbee: %s", reply);
		int sensor_number = reply[2] - '0';
		xbsocks[sensor_number] = tmpfd;
		num_detected++;
	    }
	}
	if(num_detected >= howmany)
	    break;
    }

    for(i=0;i<SENSORS;i++)
    {
	printf("%sXbee %d: ", KNRM, i);
	if(xbsocks[i] == -1)
	    printf("%sOffline\n", KRED);
	else
	    printf("%sOnline\n", KGRN);
    }
    printf("%s",KNRM);

    printf("Press enter to continue...");
    char c;
    while(c != 'c')
    {
	scanf("%c", &c);
    }
    //getch();

    return num_detected;
}
