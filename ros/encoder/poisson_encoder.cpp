#include "poisson_encoder.h"

int
main(int argc, char** argv)
{

    RateEncoder poisson_encoder;
    poisson_encoder.init(argc, argv);
    poisson_encoder.runMUSIC();
    poisson_encoder.finalize();

}

void
RateEncoder::init(int argc, char** argv)
{
    std::cout << "initializing poisson encoder" << std::endl;
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
    for (int i = 0; i < size_data; ++i)
    {
        rates[i] = 0.;
        rates_buf[i] = 0.;
        next_spike[i] = negexp(denormalize(rates[i])); 
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
    std::cout << "running poisson encoder" << std::endl;

    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    int ticks_skipped = 0;
    double t = runtime->time();

    while(t < stoptime)
    {
        t = runtime->time();

        for (int n = 0; n < size_data; ++n)
        {
            if (rates[n] != rates_buf[n])
            {
                next_spike[n] = t + negexp(denormalize(rates[n]));
                rates_buf[n] = rates[n];
            }

            while(next_spike[n] < t + timestep)
            {
#if DEBUG_OUTPUT
                std::cout << "Poisson Encoder: neuron " << n << " spiked at " << runtime->time() << std::endl;
#endif
                port_out->insertEvent(runtime->time(), MUSIC::GlobalIndex(n));
                next_spike[n] += negexp(denormalize(rates[n]));
            }
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
    std::cout << "poisson encoder: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped <<  std::endl;
}

inline double
RateEncoder::denormalize(double s)
{
    // incoming data is normalized between -1 and 1
    // 
    // returns interspike interval with rate between [min_rate, max_rate]
    
    return 1. / ((s+1) * normalization_factor + rate_min);
}

double
RateEncoder::negexp (double m)
{
    return - m * log (drand48 ());
}

void
RateEncoder::finalize(){
    runtime->finalize();
    delete runtime;
}




