#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include "wifilib.h"
#include "xbcom.h"
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
#define TESTPORT 5000
#define XBPORT 9750
#define MSGBUFSIZE 1024
#define DOF 3
#define SENSORS 7
#define TIMEOUT 2 //Seconds
#define ID_OFFSET 2
#define BASE_IP "192.168.253."
#define ITEMS 100000
#define BASE 0
#define RT 1
#define RS 2
#define RF 3
#define LT 4
#define LS 5
#define LF 6
#define PI 3.14159265
#define DEG2RAD PI/180.0
#define SENSORS_ACTIVE 2


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
	    //double q_rel[4];
	    //calc_relative_orientation(quaternions[RT], quaternions[RS], q_rel);
	    //double ypr_rel[3];
	    //quat2ypr(q_rel, ypr_rel);
	    //double temp1 = ypr_rel[1];
	    //double temp2 = sensor_readings[RS][1];
            printf("Anlge1: %.3f Angle2: %.3f  ", temp1, temp2);
	    int v,w;
	    for(w=0;w<SENSORS;w++)
	    {
		if(xbsocks[w] != 1)
		{
		    for(v=0;v<3;v++)
		    {
                        printf("%.3f ", sensor_readings[w][v]);
                        //printf("%.3f ", quaternions[w][v]);
                        logfile << quaternions[w][v] << " ";
		    }
		    printf("||");
		    logfile << "||";
		}
	    }
	    printf("\n");
	    logfile << endl;
	}
    }
    
//pthread_exit(NULL);
    return 0;
}
