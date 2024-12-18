/* Simple demo showing how to communicate with Net F/T using C language. */

#include <arpa/inet.h>
#include <netdb.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <Eigen/Dense>
#include <iostream>

#include "redis/RedisClient.h"
#include "filters/ButterworthFilter.h"
#include "tinyxml2.h"

bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

#define HEADER 0x1234
#define PORT 49152 /* Port the Net F/T always uses */
#define NUM_SAMPLES 0 // number of samples to send (0 = infinite)

using namespace std;
using namespace Eigen;

string SENSED_FORCE_KEY;
string SENSED_MOMENT_KEY;

struct RedisDriverConfig {
	std::string robot_name;
	std::string link_name;
	std::string sensor_ip_address;
	std::string redis_prefix = "sai";
	bool use_filter = false;
	double normalized_filter_cutoff_frequency = 0.05;
};

RedisDriverConfig parseRedisDriverConfig(const std::string& config_file_path) {
	RedisDriverConfig config;
	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(config_file_path.c_str()) != tinyxml2::XML_SUCCESS) {
		throw std::runtime_error("Could not load driver config file: " + config_file_path);
	}

	tinyxml2::XMLElement* driver_xml = doc.FirstChildElement("saiATIGammaDriverConfig");
	if (driver_xml == nullptr) {
		throw std::runtime_error("No 'saiATIGammaDriverConfig' element found in driver config file: " + config_file_path);
	}

	if(!driver_xml->Attribute("robotName")) {
		throw std::runtime_error("No 'robotName' attribute found in driver config file: " + config_file_path);
	}
	config.robot_name = driver_xml->Attribute("robotName");

	if(!driver_xml->Attribute("linkName")) {
		throw std::runtime_error("No 'linkName' attribute found in driver config file: " + config_file_path);
	}
	config.link_name = driver_xml->Attribute("linkName");

	if(!driver_xml->Attribute("sensorIpAddress")) {
		throw std::runtime_error("No 'sensorIpAddress' attribute found in driver config file: " + config_file_path);
	}
	config.sensor_ip_address = driver_xml->Attribute("sensorIpAddress");

	if(driver_xml->Attribute("redisPrefix")) {
		config.redis_prefix = driver_xml->Attribute("redisPrefix");
	}

	if(driver_xml->Attribute("useFilter")) {
		config.use_filter = driver_xml->BoolAttribute("useFilter");
	}

	if(driver_xml->Attribute("filterCutoff")) {
		config.normalized_filter_cutoff_frequency = driver_xml->DoubleAttribute("normalizedFilterCutoff");
	}

	return config;
}

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

	std::string config_file = "default_config.xml";
    if (argc > 1) {
        config_file = argv[1];
    }
	std::string config_file_path = std::string(CONFIG_FOLDER) + "/" + config_file;

	RedisDriverConfig config = parseRedisDriverConfig(config_file_path);

	SENSED_FORCE_KEY = "sensors::" + config.robot_name + "::ft_sensor::" + config.link_name + "::force";
	SENSED_MOMENT_KEY = "sensors::" + config.robot_name + "::ft_sensor::" + config.link_name + "::moment";

	// setup filter
	SaiCommon::ButterworthFilter filter(config.normalized_filter_cutoff_frequency);
	filter.initializeFilter(FT_eigen);

	// start redis client
	SaiCommon::RedisClient redis_client(config.redis_prefix);
	redis_client.connect();

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
	he = gethostbyname(config.sensor_ip_address.c_str());
	if (he == nullptr) {
		cout << "Host resolution failed" << endl;
		exit(1);
}
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		cout << "Connection failed" << endl;
		exit(2);
	}

	// timeout for reply
	struct timeval timeout;
	timeout.tv_sec = 5;  // 5 seconds timeout
	timeout.tv_usec = 0;
	setsockopt(socketHandle, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

	send( socketHandle, request, 8, 0 );
	
	cout << "Start streaming Force and Moment" << endl;

	runloop = true;
	while(runloop)
	{
		/* Receiving the response. */
		ssize_t bytesReceived = recv( socketHandle, response, 36, 0 );
		if (bytesReceived == -1) {
			cout << "Timed out while waiting for sensor data" << endl;
			break;
		}
		resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
		resp.ft_sequence = ntohl(*(uint32*)&response[4]);
		resp.status = ntohl(*(uint32*)&response[8]);
		for(int i = 0; i < 6; i++ ) {
			resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
			FT_eigen(i) = -resp.FTData[i] / 1e6;
		}

		Eigen::VectorXd filtered_signal = FT_eigen;
		if(config.use_filter)
		{
		    filtered_signal = filter.update(FT_eigen);
		}

		redis_client.setEigen(SENSED_FORCE_KEY, filtered_signal.head(3));
		redis_client.setEigen(SENSED_MOMENT_KEY, filtered_signal.tail(3));

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
