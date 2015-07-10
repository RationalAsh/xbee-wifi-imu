#ifndef XBCOM_H
#define XBCOM_H

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
#define PI 3.14159265
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI
#define SENSORS_ACTIVE 2
#define X 0
#define Y 1
#define Z 2
#define W 3

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
void quat2ypr(double quat[4], double ypr[3]);
void ypr2quat(double quat[4], double ypr[3]);
void quat_multiply(double quat1[4], double quat2[4], double q_res[4]);
void calc_relative_orientation(double quat1[4], double quat2[4]);

void calc_quaternions(double readings[SENSORS][DOF], double quats[SENSORS][4])
{
    int i;

    for(i=0;i<SENSORS;i++)
    {
	update_quaternion(readings, i, quats);
    }
}

void calc_relative_orientation(double quat1[4], double quat2[4], double q_rel[4])
{
    double inverse_q1[4] = {-quat1[0], -quat1[1], -quat1[2], quat1[3]};
//double relative_q[4];
    quat_multiply(inverse_q1, quat2, q_rel);
    
}

//Multiply two quaternions
void quat_multiply(double quat1[4], double quat2[4], double q_res[4])
{
    double temp1=quat1[3]*quat2[3]-quat1[0]*quat2[0]-
	         quat1[1]*quat2[1]-quat1[2]*quat2[2];
    double temp2=quat1[3]*quat2[0]+quat2[3]*quat1[0]+
	         quat1[1]*quat2[2]-quat2[1]*quat1[2];
    double temp3=quat1[3]*quat2[1]+quat2[3]*quat1[1]+
                 quat1[2]*quat2[0]-quat2[2]*quat1[0];
    double temp4=quat1[3]*quat2[2]+quat2[3]*quat1[2]+
	         quat1[0]*quat2[1]-quat2[0]*quat1[1];

        
    q_res[0] = temp2;
    q_res[1] = temp3;
    q_res[2] = temp4;
    q_res[3] = temp1;
}

//Converts quaternions to yaw/pitch/roll values
void quat2ypr(double q[4], double ypr[3])
{
/*
    heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy^2 - 2*qz^2)
    attitude = asin(2*qx*qy + 2*qz*qw)
    bank = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx2 - 2*qz2)

    except when qx*qy + qz*qw = 0.5 (north pole)
    which gives:
    heading = 2 * atan2(x,w)
    bank = 0
    and when qx*qy + qz*qw = -0.5 (south pole)
    which gives:
    heading = -2 * atan2(x,w)
    bank = 0
*/

    double yaw = atan2(2*q[Y]*q[W] - 2*q[X]*q[Z], 1-2*q[Y]*q[Y]-2*q[Z]*q[Z]);
    double pitch = asin(2*q[X]*q[Y] + 2*q[Z]*q[W]);
    double roll = atan2(2*q[X]*q[W] - 2*q[Y]*q[Z], 1-2*q[X]*q[X]-2*q[Z]*q[Z]);
    ypr[0] = yaw*RAD2DEG;
    ypr[1] = pitch*RAD2DEG;
    ypr[2] = roll*RAD2DEG;
}

//Converts yaw/pitch/roll to quaternions
void ypr2quat(double q[4], double ypr[3])
{ 
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

    //double norm = sqrt(w*w + x*x + y*y + z*z);

    q[0] = x;
    q[1] = y;
    q[2] = z;
    q[3] = w;
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
void Quaternion_Multiple(double *a, double *b)
{
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


#endif /* XBCOM_H */
