#include "ros_sensor_adapter.h"

#include "rtclock.h"

static void*
ros_thread(void* arg)
{
    RosSensorAdapter* ros_adapter = static_cast<RosSensorAdapter*>(arg);
    ros_adapter->runROS();
}

int
main(int argc, char** argv)
{

    RosSensorAdapter ros_adapter;
    ros_adapter.init(argc, argv);

    MPI::COMM_WORLD.Barrier();
    // If sensor_update_rate and timestep match to a relative
    // precision of 0.1%, lump the ROS and MUSIC event loops
    // together.
    if (ros_adapter.ratesMatch (0.001))
    {
	    ros_adapter.runROSMUSIC();
    }
    else
    {
        pthread_t t;
	    pthread_create (&t, NULL, ros_thread, &ros_adapter);

    	ros_adapter.runMUSIC();
    	pthread_join(t, NULL);
    }

    ros_adapter.finalize();

}


bool
RosSensorAdapter::ratesMatch (double precision)
{
    return std::abs (sensor_update_rate * timestep - 1.) < precision;
}


void
RosSensorAdapter::init(int argc, char** argv)
{
    std::cout << "initializing ROS sensor adapter" << std::endl;

    timestep = DEFAULT_TIMESTEP;
    sensor_update_rate = DEFAULT_SENSOR_UPDATE_RATE;
    ros_node_name = DEFAULT_ROS_NODE_NAME;
    rtf = DEFAULT_RTF;

    pthread_mutex_init(&data_mutex, NULL);

    // MUSIC before ROS to read the config first!
    initMUSIC(argc, argv);
    initROS(argc, argv);
}


void
RosSensorAdapter::initROS(int argc, char** argv)
{
    ros::init(argc, argv, ros_node_name);
    ros::start();

    ros::NodeHandle n;
    switch (msg_type)
    {
        case Laserscan:
            subscriber = n.subscribe(ros_topic, 1000, &RosSensorAdapter::laserscanCallback, this);
            break;
        case Twist:
            subscriber = n.subscribe(ros_topic, 1000, &RosSensorAdapter::twistCallback, this);
            break;
        case Float64MultiArray:
            subscriber = n.subscribe(ros_topic, 1000, &RosSensorAdapter::float64MultiArrayCallback, this);
            break;
    }
}

void
RosSensorAdapter::initMUSIC(int argc, char** argv)
{
    setup = new MUSIC::Setup (argc, argv);

    setup->config("ros_topic", &ros_topic);
    setup->config("stoptime", &stoptime);
    setup->config("sensor_update_rate", &sensor_update_rate);
    setup->config("ros_node_name", &ros_node_name);
    setup->config("rtf", &rtf);

    std::string _msg_type;
    setup->config("message_type", &_msg_type);

    if (_msg_type.compare("Laserscan") == 0){
        msg_type = Laserscan;
    }
    else if (_msg_type.compare("Twist") == 0){
        msg_type = Twist;
    }
    else if (_msg_type.compare("FloatArray") == 0){
        msg_type = Float64MultiArray;
    }
    else
    {
        std::cout << "ERROR: msg type unknown" << std::endl;
        finalize();
    }


    MUSIC::ContOutputPort* port_out = setup->publishContOutput ("out");

    comm = setup->communicator ();
    int rank = comm.Get_rank ();       
    int nProcesses = comm.Get_size (); 
    if (nProcesses > 1)
    {
        std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
        comm.Abort(1);
    }


    if (port_out->hasWidth ())
    {
        datasize = port_out->width ();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort (1);
    }
    
    data = new double[datasize]; 
    for (unsigned int i = 0; i < datasize; ++i)
    {
        data[i] = 0.;
    }
         
    // Declare where in memory to put data
    MUSIC::ArrayData dmap (data,
      		 MPI::DOUBLE,
      		 0,
      		 datasize);
    port_out->map (&dmap, 1);
}

void
RosSensorAdapter::runROSMUSIC()
{
    std::cout << "running sensor adapter with update rate of " << sensor_update_rate << std::endl;
    RTClock clock( 1. / (sensor_update_rate * rtf) );
    
    ros::spinOnce();
    runtime = new MUSIC::Runtime (setup, timestep);

    for (int t = 0; runtime->time() < stoptime; t++)
    {

#if DEBUG_OUTPUT
        std::cout << "ROS Sensor Adapter: ";
        for (int i = 0; i < datasize; ++i)
        {
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;
#endif

        clock.sleepNext(); 
        ros::spinOnce();
        runtime->tick();
    }

    std::cout << "sensor: total simtime: " << clock.time () << " s" << std::endl;
}

void
RosSensorAdapter::runROS()
{
    RTClock clock( 1. / (sensor_update_rate * rtf) );

    // wait until first sensor update arrives
    while (ros::Time::now().toSec() == 0.)
    {
        clock.sleepNext();
    }

    ros::Time stop_time = ros::Time::now() + ros::Duration(stoptime/rtf);

    ros::spinOnce();
    for (ros::Time t = ros::Time::now(); t < stop_time; t = ros::Time::now())
    {
#if DEBUG_OUTPUT
        std::cout << "ROS Sensor Adapter: ";
        for (int i = 0; i < datasize; ++i)
        {
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;
#endif

        clock.sleepNext();
        ros::spinOnce();
   }
}

void 
RosSensorAdapter::runMUSIC()
{
    std::cout << "running sensor adapter with update rate of " << sensor_update_rate << std::endl;
    RTClock clock(timestep / rtf);

    runtime = new MUSIC::Runtime (setup, timestep);
    
    for (int t = 0; runtime->time() < stoptime; t++)
    {
        clock.sleepNext(); 
    	pthread_mutex_lock(&data_mutex);
        runtime->tick();
	    pthread_mutex_unlock(&data_mutex);
    }

    std::cout << "sensor: total simtime: " << clock.time () << " s" << std::endl;
}

void
RosSensorAdapter::laserscanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    pthread_mutex_lock(&data_mutex);
    for (unsigned int i = 0; i < msg->ranges.size(); ++i)
    {
        // scale data between -1 and 1
        // TODO: catch exception if ranges.size not width of port
        if (isinf(msg->ranges.at(i))){
            data[i] = 1.;
        }
        else{
            data[i] = ((msg->ranges.at(i) - msg->range_min) / (msg->range_max - msg->range_min) ) * 2 - 1;
        }
    }
    pthread_mutex_unlock(&data_mutex);    
}

void
RosSensorAdapter::twistCallback(const geometry_msgs::Twist msg)
{
    pthread_mutex_lock(&data_mutex);

    data[0] = msg.linear.x;
    data[1] = msg.angular.z;
    for (unsigned int i = 0; i < 2; ++i) // Twist msg has 2 dimensions
    {
        // limit data between -1 and 1
        if (data[i] > 1)
            data[i] = 1;
        else if (data[i] < -1)
            data[i] = -1;

    }

    pthread_mutex_unlock(&data_mutex);    
}

void
RosSensorAdapter::float64MultiArrayCallback(const std_msgs::Float64MultiArray msg)
{
    pthread_mutex_lock(&data_mutex);

    for (unsigned int i = 0; i < datasize; ++i)
    {
        data[i] = msg.data[i];
    }

    pthread_mutex_unlock(&data_mutex);    
}

void RosSensorAdapter::finalize(){
    runtime->finalize();
    delete runtime;
}



