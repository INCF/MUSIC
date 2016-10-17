#include "rate_encoder.h"

int
main(int argc, char** argv)
{

    RateEncoder rate_encoder;
    rate_encoder.init(argc, argv);
    rate_encoder.runMUSIC();
    rate_encoder.finalize();

}

void
RateEncoder::init(int argc, char** argv)
{
    std::cout << "initializing rate encoder" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    rate_min = DEFAULT_RATE_MIN;
    rate_max = DEFAULT_RATE_MAX;
    initMUSIC(argc, argv);
}

void
RateEncoder::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);
    setup->config("rate_min", &rate_min);
    setup->config("rate_max", &rate_max);
    normalization_factor = (rate_max - rate_min) / 2.; 

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
        size_data = port_in->width();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort(1);
    }
    
    rates = new double[size_data];
    rates_buf = new double[size_data];
    next_spike = new double[size_data];
    last_spike = new double[size_data];
    for (int i = 0; i < size_data; ++i)
    {
        rates[i] = -1.;
        rates_buf[i] = -1.;
        last_spike[i] = 0.;
        next_spike[i] = rate2SpikeTime(rates[i]); 
    }
         
    // Declare where in memory to put sensor_data
    MUSIC::ArrayData dmap(rates,
      		 MPI::DOUBLE,
      		 0,
      		 size_data);
    port_in->map (&dmap, 0., 1, false);
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index_out(0, size_data);
    port_out->map(&l_index_out, MUSIC::Index::GLOBAL, 1);

    MPI::COMM_WORLD.Barrier();
    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
RateEncoder::runMUSIC()
{
    std::cout << "running rate encoder" << std::endl;

    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    int ticks_skipped = 0;
    unsigned int num_spikes = 0;

    double t = runtime->time();
    while(t < stoptime)
    {
        t = runtime->time();

        double next_t = t + timestep;
        for (int n = 0; n < size_data; ++n)
        {
            if (rates[n] != rates_buf[n])
            {
                double old_isi = next_spike[n] - last_spike[n];
                double part_time_left = (t - last_spike[n]) / old_isi; 
                double new_isi = rate2SpikeTime(rates[n]) * (1 - part_time_left);
                next_spike[n] = t + new_isi;
                rates_buf[n] = rates[n];
            }


            while(next_spike[n] < next_t)
            {
#if DEBUG_OUTPUT
                std::cout << "Rate Encoder: neuron " << n << " spikes at " << next_spike[n] << " simtime: " << t << " rate " << rates[n] << std::endl;
#endif
                num_spikes++;
                port_out->insertEvent(next_spike[n], MUSIC::GlobalIndex(n));
                last_spike[n] = next_spike[n];
                next_spike[n] += rate2SpikeTime(rates[n]); 
            }
        }
//#if DEBUG_OUTPUT
//        std::cout << "Rate Encoder: ";
//        for (int i = 0; i < size_data; ++i)
//        {
//            std::cout << rates[i] << " " << rates_buf[i] << " ";
//        }
//        std::cout << std::endl;
//#endif

        runtime->tick();
    }

    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "rate encoder: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped << " num spikes " << num_spikes << std::endl;
}

double
RateEncoder::rate2SpikeTime(double r)
{
    // the incoming data, which is interpreted as rate, is between -1 and 1.
    //
    // scales rate between [rate_min, rate_max]
    //
    // returns next spike time
    return 1. / ((r+1) * normalization_factor + rate_min);
}

void
RateEncoder::finalize(){
    runtime->finalize();
    delete runtime;
}




