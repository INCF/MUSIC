#include <iostream>
#include <map>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

#include <music.hh>
#include <mpi.h>

#include "sys/time.h"

#include "jsoncpp/json/json.h"
#include <fstream>
#include <pthread.h>

#define DEBUG_OUTPUT false 
#define MEASUREMENT_OUTPUT false 

enum msg_types {Float64MultiArray, Twist};

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_COMMAND_RATE = 10;
const double DEFAULT_RTF = 1.0;
const msg_types DEFAULT_MESSAGE_TYPE = Float64MultiArray;

class RosCommandAdapter
{
    public:
        void init(int argc, char** argv);
        bool ratesMatch (double precision);
        void runMUSIC();
        void runROS();
	    void runROSMUSIC();
        void finalize();

    private:
        std::string ros_topic;
        ros::Publisher publisher;

        MPI::Intracomm comm;
	    MUSIC::Setup* setup;
        MUSIC::Runtime* runtime;
        double stoptime;
        int datasize;

    	pthread_mutex_t data_mutex;
        double* data;

        double timestep;
        double command_rate;
        double rtf;

        string mapping_filename;
        Json::Value json_mapping; 
        msg_types msg_type;
        int* msg_map;

        void initROS(int argc, char** argv);
        void initMUSIC(int argc, char** argv);
	    void sendROS();

        void readMappingFile();

#if MEASUREMENT_OUTPUT
        void saveRuntime(double rt);
#endif
};
