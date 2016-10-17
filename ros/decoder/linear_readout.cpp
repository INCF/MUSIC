#include "linear_readout.h"

int
main(int argc, char** argv)
{

    LinearReadoutDecoder lin_decoder;
    lin_decoder.init(argc, argv);
    lin_decoder.runMUSIC();
    lin_decoder.finalize();

}

void
LinearReadoutDecoder::init(int argc, char** argv)
{
    std::cout << "initializing linear readout decoder" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    weights_filename = DEFAULT_WEIGHTS_FILENAME;
    tau = DEFAULT_TAU;
    num_spikes0 = 0;

    // init MUSIC to read config
    initMUSIC(argc, argv); 
    readWeightsFile();
}

void
LinearReadoutDecoder::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);
    setup->config("weights_filename", &weights_filename);
    setup->config("tau", &tau);
    inv_tau = 1. / tau;
    acceptable_latency = timestep;

    port_in = setup->publishEventInput("in");
    port_out = setup->publishContOutput("out");

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
        size_spike_data = port_in->width();
        size_command_data = port_out->width();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort(1);
    }
    
    command_data = new double[size_command_data];
    for (int i = 0; i < size_command_data; ++i)
    {
        command_data[i] = 0.;
    }
    vec_command_data = gsl_vector_view_array(command_data, size_command_data);

    activity_traces = new double[size_spike_data];
    for (int i = 0; i < size_spike_data; ++i)
    {
        activity_traces[i] = 0.;
    }
    vec_activity_traces = gsl_vector_view_array(activity_traces, size_spike_data);

    readout_weights = new double[size_command_data * size_spike_data];
    for (int i = 0; i < size_command_data * size_spike_data; ++i)
    {
        readout_weights[i] = 0.;
    }
    mat_readout_weights = gsl_matrix_view_array(readout_weights, size_command_data, size_spike_data);
         
    // Declare where in memory to put command_data
    MUSIC::ArrayData dmap(command_data,
			  MPI::DOUBLE,
			  0,
			  size_command_data);
    port_out->map (&dmap, 1);
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index_in(0, size_spike_data);
    port_in->map(&l_index_in, this, acceptable_latency, 1); 

    // initialize propagator for exponential decay
    propagator = std::exp(-timestep/tau);

    MPI::COMM_WORLD.Barrier();
    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
LinearReadoutDecoder::readWeightsFile()
{
    Json::Reader json_reader;

    std::ifstream weights_file;
    weights_file.open(weights_filename.c_str(), std::ios::in);
    string json_weights = "";
    string line;

    while (std::getline(weights_file, line))
    {
        json_weights += line;
    }
    weights_file.close();
    
    if ( !json_reader.parse(json_weights, json_readout_weights))
    {
        // report to the user the failure and their locations in the document.
        std::cout   << "WARNING: linear readout: Failed to parse file \"" << weights_filename << "\"\n" 
                    << json_weights << " It has to be in JSON format.\n Using 1/N for each weight."
                    << json_reader.getFormattedErrorMessages();
        
        for (int i = 0; i < size_command_data * size_spike_data; ++i)
        {
            readout_weights[i] = 1. / size_spike_data;
        }

        return;
    }
    else
    {
        for (int i = 0; i < size_command_data; ++i)
        {
            for (int j = 0; j < size_spike_data; ++j)
            {
                readout_weights[i * size_spike_data + j] = json_readout_weights[i][j].asDouble();
            }
        }

    }

}

void 
LinearReadoutDecoder::runMUSIC()
{
    std::cout << "running linear readout decoder" << std::endl;

    int num_spikes_decoded = 0;
    
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    
    double t = runtime->time();
    for (t = runtime->time (); t < stoptime; t = runtime->time ())
    {
        double next_t = t + timestep;
        while (!spikes.empty () && spikes.top ().t < next_t)
        {
            double t_spike = spikes.top ().t;
            int id = spikes.top ().id;
            
            activity_traces[id] += (std::exp ((t_spike - t) * inv_tau) * inv_tau);
            
            spikes.pop (); // remove spike from queue
            ++num_spikes_decoded;
        }

        for (int j = 0; j < size_spike_data; ++j)
        {
            activity_traces[j] *= propagator; // decay
        }

        gsl_blas_dgemv(CblasNoTrans, 1., &mat_readout_weights.matrix, &vec_activity_traces.vector, 0., &vec_command_data.vector);

#if DEBUG_OUTPUT
        std::cout << "Linear Readout: Activity Traces: ";
        for (int i = 0; i < size_spike_data; ++i)
        {
            std::cout << activity_traces[i] << " ";
        }
        std::cout << std::endl;


        std::cout << "Linear Readout: Command Data: ";
        for (int i = 0; i < size_command_data; ++i)
        {
            std::cout << command_data[i] << " ";
        }
        std::cout << std::endl;
#endif
        runtime->tick();
    }
    
    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "decoder: total simtime: " << dt_s << " " << dt_us << " received spikes " << num_spikes0 << " spikes decoded " << num_spikes_decoded << std::endl;
    
}

void LinearReadoutDecoder::operator () (double t, MUSIC::GlobalIndex id){
    // Decoder: add incoming spikes to map
    num_spikes0++;

    spikes.push (Event (t + acceptable_latency, id));
}
void LinearReadoutDecoder::finalize(){
    runtime->finalize();
    delete runtime;
}
