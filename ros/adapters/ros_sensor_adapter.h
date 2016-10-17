#include <iostream>
#include <map>
#include <math.h>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

#include <music.hh>
#include <mpi.h>

#include "sys/time.h"

#include <iostream>

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_SENSOR_UPDATE_RATE = 30;
const double DEFAULT_RTF = 1.0;
const std::string DEFAULT_ROS_NODE_NAME = "ros_sensor_node";

enum msg_types {Laserscan, Twist, Float64MultiArray}; 

class RosSensorAdapter
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
        ros::Subscriber subscriber;
        std::string ros_node_name;
        double rtf;

        MPI::Intracomm comm;
	    MUSIC::Setup* setup;
        MUSIC::Runtime* runtime;
        double stoptime;
        int datasize;
        double sensor_update_rate;
        double timestep;

	    pthread_mutex_t data_mutex;
        double* data;

        msg_types msg_type;

        void initROS(int argc, char** argv);
        void initMUSIC(int argc, char** argv);

        void laserscanCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void twistCallback(const geometry_msgs::Twist msg);
        void float64MultiArrayCallback(const std_msgs::Float64MultiArray msg);

};
