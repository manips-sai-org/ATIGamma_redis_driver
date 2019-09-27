/* Simple demo showing how to communicate with Net F/T using C language. */

#include <Eigen/Dense>
#include <iostream>
#include "RedisClient.h"

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "filters/ButterworthFilter.h"

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

#define HEADER 0x1234
#define PORT 49152 /* Port the Net F/T always uses */
#define NUM_SAMPLES 0 // number of samples to send (0 = infinite)

using namespace std;
using namespace Eigen;

string SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::force_torque";

sai::ButterworthFilter filter;
const double cutoff_freq = 0.05;  //the cutoff frequency of the filter, in the range of (0 0.5) of sampling freq
bool use_filter = false;


/* Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;

typedef struct response_struct {
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} RESPONSE;

enum COMMAND {
	STOP_STREAMING              =  0,
	START_HIGH_SPEED_STREAMING  =  2,    // for realtime applications
	START_BUFFERED_STREAMING    =  3,    // for data collection
	START_MULTI_UNIT_STREAMING  =  3,    // for synchronisation between several sensors
	RESET_THRESHOLD_LATCH       = 41,    // if threshold monitoring is enabled
	SET_SOFTWARE_BIAS           = 42    
};

unsigned long long counter = 0;

int main ( int argc, char ** argv ) {
	int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
	struct sockaddr_in addr;	/* Address of Net F/T. */
	struct hostent *he;			/* Host entry for Net F/T. */
	byte request[8];			/* The request data sent to the Net F/T. */
	RESPONSE resp;				/* The structured response received from the Net F/T. */
	byte response[36];			/* The raw response data received from the Net F/T. */
	// int i;						/* Generic loop/array index. */
	int err;					/* Error status of operations. */
	string AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };	/* The names of the force and torque axes. */

	VectorXd FT_eigen = VectorXd::Zero(6);

	if ( 2 > argc )
	{
		fprintf( stderr, "Usage: %s IPADDRESS\n", argv[0] );
		return -1;
	}
	
	std::string sensor_ip = argv[1];

	if(sensor_ip == "172.16.0.20")
	{
		SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::Clyde::force_torque";
	}
	else if(sensor_ip == "172.16.0.21")
	{
		SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::Bonnie::force_torque";
	}

    if(use_filter)
    {
        filter.setDimension(6);
        filter.setCutoffFrequency(cutoff_freq);
    }

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = CDatabaseRedisClient();
	redis_client.serverIs(info);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	/* Calculate number of samples, command code, and open socket here. */
	socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1) {
		exit(1);
	}
	
	*(uint16*)&request[0] = htons(HEADER);                         /* standard header. */
	*(uint16*)&request[2] = htons(START_HIGH_SPEED_STREAMING);     /* per table 9.1 in Net F/T user manual. */
	*(uint32*)&request[4] = htonl(NUM_SAMPLES);                    /* see section 9.1 in Net F/T user manual. */
	
	/* Sending the request. */
	he = gethostbyname(argv[1]);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		exit(2);
	}
	send( socketHandle, request, 8, 0 );
	
	cout << "Start streaming Force and Moment" << endl;

	runloop = true;
	while(runloop)
	{
		/* Receiving the response. */
		recv( socketHandle, response, 36, 0 );
		resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
		resp.ft_sequence = ntohl(*(uint32*)&response[4]);
		resp.status = ntohl(*(uint32*)&response[8]);
		for(int i = 0; i < 6; i++ ) {
			resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
			FT_eigen(i) = -resp.FTData[i] / 1e6;
		}

		Eigen::VectorXd filtered_signal = FT_eigen;
		if(use_filter)
		{
		    filtered_signal = filter.update(FT_eigen);
		}


		redis_client.setEigenMatrixDerived(SENSED_FORCE_KEY, filtered_signal);

		// /* Output the response data. */
		// cout << "Status : " << resp.status << endl;
		// cout << FT_eigen.transpose() << endl;
		// cout << endl;

		counter++;
	}

	// stop streaming before exiting
	cout << "\nExiting Gamma driver and stop streaming" << endl;
	*(uint16*)&request[2] = htons(STOP_STREAMING);
	send( socketHandle, request, 8, 0 );

	return 0;
}
