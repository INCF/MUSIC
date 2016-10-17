#include <music.hh>
#include <mpi.h>

#include <vector>
#include <cmath>
#include <unistd.h>
#include "sys/time.h"

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_RATE_MIN = 0.;
const double DEFAULT_RATE_MAX = 100.;

class RateEncoder{
    public:
        void init(int argc, char** argv);
        void runMUSIC();
        void finalize();

    private:
        MPI::Intracomm comm;
        MUSIC::Runtime* runtime;
        double stoptime;
        double timestep;
        int size_data;
        double normalization_factor;

        double rate_min, rate_max;
        double* next_spike;
        double* rates;
        double* rates_buf;
        MUSIC::EventOutputPort* port_out;
        MUSIC::ContInputPort* port_in;

        void initMUSIC(int argc, char** argv);
        double denormalize(double s);
        double negexp (double m);
};


