#include "nef_encoder.h"

int
main(int argc, char** argv)
{

    NefEncoder nef_encoder;
    nef_encoder.init(argc, argv);
    nef_encoder.runMUSIC();
    nef_encoder.finalize();

}

void
NefEncoder::init(int argc, char** argv)
{
    std::cout << "initializing nef encoder" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    initMUSIC(argc, argv);
}

void
NefEncoder::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);

    port_in = setup->publishContInput("in");
    port_out = setup->publishEventOutput("out");

    comm = setup->communicator ();
    int rank = comm.Get_rank ();       
    int nProcesses = comm.Get_size ();
    if (nProcesses > 1)
    {
        std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
        comm.Abort(1);
    }

    // get dimensions of sensory data and spike data
    if (port_in->hasWidth() && port_out->hasWidth())
    {
        size_sensor_data = port_in->width();
        size_spike_data = port_out->width();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort(1);
    }
    
    for (unsigned int i = 0; i < size_sensor_data; ++i)
    {
        sensor_data.push_back(0.);
        sensor_data_buf.push_back(0.);
    }

    for (int n = 0; n < size_spike_data; ++n){
        IAFNeuron neuron(sensor_data.size());
        neuron.setResolution(DEFAULT_NEURON_RESOLUTION);
        neurons.push_back(neuron);
        neuron.encode(sensor_data);
    }

         
    // Declare where in memory to put sensor_data
    MUSIC::ArrayData dmap(&sensor_data[0],
      		 MPI::DOUBLE,
      		 0,
      		 size_sensor_data);
    port_in->map (&dmap, 0., 1, false);
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index_out(0, size_spike_data);
    port_out->map(&l_index_out, MUSIC::Index::GLOBAL, 1);

    MPI::COMM_WORLD.Barrier();
    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
NefEncoder::runMUSIC()
{
    std::cout << "running nef encoder" << std::endl;

    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    int ticks_skipped = 0;
    int _ticks_to_skip = 0;

    double t = runtime->time();
    while(t < stoptime)
    {
        t = runtime->time();

        if (sensor_data != sensor_data_buf) 
        {
            for (unsigned int n = 0; n < neurons.size(); ++n)
            {
                neurons[n].encode(sensor_data);
            }
            sensor_data_buf = sensor_data;

#if DEBUG_OUTPUT
            for (unsigned int n = 0; n < sensor_data.size(); ++n)
            {
                std::cout << sensor_data[n] << " ";
            }
            std::cout << std::endl;
#endif
        }

        double next_t = t + timestep;
        while (t <= next_t)
        {
            for (unsigned int n = 0; n < neurons.size(); ++n)
            {
                if (neurons[n].propagate())
                {
#if DEBUG_OUTPUT
//                    std::cout << "NEF Encoder: neuron " << n << " spiked at " << runtime->time() << std::endl;
#endif
                    port_out->insertEvent(runtime->time(), MUSIC::GlobalIndex(n));
                }
            }
            t += DEFAULT_NEURON_RESOLUTION; // propagate a ms
        }
       
        runtime->tick();
    }

    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "encoder: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped <<  std::endl;
}

void NefEncoder::finalize(){
    runtime->finalize();
    delete runtime;
}




