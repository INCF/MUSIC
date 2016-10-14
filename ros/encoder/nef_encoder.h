#include <music.hh>
#include <mpi.h>

#include <vector>
#include <cmath>
#include <unistd.h>
#include "sys/time.h"

#include "iaf_neuron.h"

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_NEURON_RESOLUTION = 1e-3;

class NefEncoder{
    public:
        void init(int argc, char** argv);
        void runMUSIC();
        void finalize();

    private:
        MPI::Intracomm comm;
        MUSIC::Runtime* runtime;
        double stoptime;
        double timestep;
        int size_sensor_data;
        int size_spike_data;
        std::vector<double> sensor_data;
        std::vector<double> sensor_data_buf;
        std::vector<IAFNeuron> neurons;
        MUSIC::EventOutputPort* port_out;
        MUSIC::ContInputPort* port_in;

        void initMUSIC(int argc, char** argv);
};


