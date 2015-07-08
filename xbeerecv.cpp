#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include "wifilib.h"
#include <iostream>
#include <fstream>
using namespace std;
//#include "HMMgait.h"

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
#define ITEMS 100000
#define RT 2
#define RS 3
#define LT 3
#define LS 4
#define PI 3.14159265
#define DEG2RAD PI/180.0
#define SENSORS_ACTIVE 6


//HMM stuff
// observation obs[ITEMS];
// observation* ptrobs;
// obslikihood obslik[ITEMS];
// obslikihood* obslikptr;
// obslikihood* obslikstart = obslik;

//Sensor readings stuff
double sensor_readings[SENSORS][DOF];
double quaternions[SENSORS][4];
int sensormask[SENSORS] = {0,0,0,0,0,0,0};

//Structs for IO waiting
fd_set master;

int find_sensors(char* base_ip, int start, int end, int howmany, 
		 int xbsocks[SENSORS], char xbips[SENSORS][16]);
void send_start_signal(int xbsocks[SENSORS], char *c);
int read_data(int xbsocks[SENSORS], double readings[SENSORS][DOF]);
void update_ypr(char *data_str, int sensor_number, double readings[SENSORS][DOF]);
void print_readings(double readings[SENSORS][DOF]);
void print_quaternions(double quats[SENSORS][4]);
void update_quaternion(double readings[SENSORS][DOF], 
		       int sensor_number, double quats[SENSORS][4]);
void calc_quaternions(double readings[SENSORS][DOF], double quats[SENSORS][4]);
void monitor_connections(double readings[SENSORS][DOF], 
			 double xbsocks[SENSORS], double xbips[SENSORS][16]);
void Quaternion_Multiple(double *a, double *b);
double quat2angleY(double *q);

void *test_thread(void *tid)
{
    int i;
    for(i=0;i<10;i++)
    {
	printf("Hello world! %d",i);
    }
    pthread_exit(NULL);
}

int main(int argc, char** argv)
{
    int xbsocks[SENSORS];
    char xbips[SENSORS][16];
    int i;
    pthread_t test_t;
    ofstream logfile;
    logfile.open("log.txt", ios::app);

    //Clearing the socket array
    for(i=0;i<SENSORS;i++)
	xbsocks[i] = -1;

    //pthread_create(&test_t, NULL, test_thread, (void *)i);

    /*
      Scan all the IP addresses from BASE_IP
      Find and connect to the sensors and store
      the scoket descriptors in an array
    */
    //int num_found = find_sensors("172.23.38.",100, 200, 2, xbsocks, xbips);
    int num_found = find_sensors("192.168.253.",100, 200, SENSORS_ACTIVE, xbsocks, xbips);

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
//print_readings(sensor_readings);
	    //Convert to quaternions and store in quaternion array
	    calc_quaternions(sensor_readings, quaternions);
            //print_quaternions(quaternions);
	    double invqRT[] = {-quaternions[RT][0],-quaternions[RT][1],
				   -quaternions[RT][2],quaternions[RT][3]};
	    Quaternion_Multiple(invqRT, quaternions[RS]);
	    double temp1 = quat2angleY(invqRT);
            double temp2 = quat2angleY(quaternions[RS]);
            printf("Anlge1: %.3f Angle2: %.3f  ", temp1, temp2);
	    int v,w;
	    for(w=0;w<SENSORS;w++)
	    {
		if(xbsocks[w] != 1)
		{
		    for(v=0;v<3;v++)
		    {
			printf("%.3f ", sensor_readings[w][v]);
                        logfile << sensor_readings[w][v] << " ";
		    }
		    printf("||");
		    logfile << "||";
		}
	    }
	    printf("\n");
	    logfile << endl;
	}
    }
    
    pthread_exit(NULL);
    return 0;
}

void monitor_connections(double readings[SENSORS][DOF], 
			 double xbsocks[SENSORS], char 
			 xbips[SENSORS][16])
{
    int i, j;
    double monitor[SENSORS] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    static double prev_readings[SENSORS][DOF];
    static int same_count[SENSORS] = {0,0,0};
   

    //Check if current readings are the same
    //as previous readings
    for(i=0;i<SENSORS;i++)
    {
	if(abs(readings[i][0] - prev_readings[i][0])+
	   abs(readings[i][1] - prev_readings[i][1])+
	   abs(readings[i][2] - prev_readings[i][2]) == 0)
	    (same_count[SENSORS])++;
    }
    

    //If the readings have not changed for a while
    //Try to reconnect
    for(i=0;i<SENSORS;i++)
    {
	if(same_count[i] > 10)
	{
	    //Try to reconnect to this sensor
	    close(xbsocks[i]);
	    xbsocks[i] = establishConnection(xbips[i], XBPORT);
	    //int find_sensors(BASE_IP, 0, 255, 1, xbsocks, xbips);
	}
    }
	

    //Update the previous readings buffer
    for(i=0;i<SENSORS;i++)
    {
	for(j=0;j<DOF;j++)
	{
	    prev_readings[i][j] = readings[i][j];
	}
    } 
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
    
    double c1 = cos(DEG2RAD * ypr[0]/2.0);
    double c2 = cos(DEG2RAD * ypr[1]/2.0);
    double c3 = cos(DEG2RAD * ypr[2]/2.0);
    double s1 = sin(DEG2RAD * ypr[0]/2.0);
    double s2 = sin(DEG2RAD * ypr[1]/2.0);
    double s3 = sin(DEG2RAD * ypr[2]/2.0);

    double w = c1*c2*c3 - s1*s2*s3;
    double x = s1*s2*c3 + c1*c2*s3;
    double y = s1*c2*c3 + c1*s2*s3;
    double z = c1*s2*c3 - s1*c2*s3;

    double norm = sqrt(w*w + x*x + y*y + z*z);

    quats[sensor_number][0] = x/norm;
    quats[sensor_number][1] = y/norm;
    quats[sensor_number][2] = z/norm;
    quats[sensor_number][3] = w/norm;    
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
		    //printf("Xbee%d: %s\n", i, reply);
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

int find_sensors(char* base_ip, int start, int end, int howmany, int xbsocks[SENSORS], char xbips[SENSORS][16])
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

    printf("Searching for sensors\n");

    for(i=start;i<=end;i++)
    {
	sprintf(tmp, "%d", i);
	sprintf(tmpip, "%s%s", base_ip, tmp);
	printf("Trying IP: %s\n", tmpip);
	//printf(" . ");
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
		printf("Found an xbee: %s", reply);
		int sensor_number = reply[2] - '0';
		xbsocks[sensor_number] = tmpfd;
		sprintf(xbips[sensor_number], "%s%s",base_ip, tmp);
		num_detected++;
	    }
	}
	if(num_detected >= howmany)
	    break;
    }

    printf("\nThe following Xbees were detected:\n");

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
    //while(c != 'c')
    //{
	scanf("%c", &c);
	//printf("%d", c);
	//}
    //getch();

    return num_detected;
}

//Quaternion operation stuff
// quaternion multiplication
void Quaternion_Multiple(double *a, double *b){
	double temp1=a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2];
	double temp2=a[3]*b[0]+b[3]*a[0]+a[1]*b[2]-b[1]*a[2];
	double temp3=a[3]*b[1]+b[3]*a[1]+a[2]*b[0]-b[2]*a[0];
	double temp4=a[3]*b[2]+b[3]*a[2]+a[0]*b[1]-b[0]*a[1];

	a[0] = temp2;
	a[1] = temp3;
	a[2] = temp4;
	a[3] = temp1;

}

// return euler angle in y-axis
double quat2angleY(double *q)	
{
	
//	double r11, r12, r21, r31, r32;
//	r11 = 2 * (q[0] * q[1] + q[3] * q[2]);
//	r12 = pow(q[3], 2) + pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2);
//	r21 = -2 * (q[0] * q[2] - q[3] * q[1]);
//	r31 = 2 * (q[1] * q[2] + q[3] * q[0]);
//	r32 = pow(q[3], 2) - pow(q[0], 2) - pow(q[1], 2) + pow(q[2], 2);

//	a[2] = atan2(r11, r12);
//	a[1] = asin(r21);
//	a[0] = atan2(r31, r32);

	return  asin(-2 * (q[0] * q[2] - q[3] * q[1])) * (-57.29578);		// *180/PI = * 57.29578	
}
