#!/usr/bin/python

import nest
import numpy as np
import sys
from datetime import datetime
from optparse import OptionParser
from mpi4py import MPI

comm = MPI.COMM_WORLD

to_ms = lambda t: t * 1000.

opt_parser = OptionParser()
opt_parser.add_option("-t", "--simtime", dest="simtime", type="float", help="Simulation time in s")
opt_parser.add_option("-s", "--timestep", dest="music_timestep", type="float", help="MUSIC timestep")
opt_parser.add_option("-n", "--num_neurons", dest="num_neurons", type="int", help="Number of encoding neurons")

(options, args) = opt_parser.parse_args()

nest.ResetKernel()
nest.set_verbosity("M_FATAL")
nest.SetKernelStatus({'resolution': 1.0})
#nest.SetKernelStatus({'print_time': True})

NUM_ENC_NEURONS = options.num_neurons 

proxy_in = nest.Create('music_event_in_proxy', NUM_ENC_NEURONS)
nest.SetStatus(proxy_in, [{'port_name': 'in', 'music_channel': c} for c in range(NUM_ENC_NEURONS)])
nest.SetAcceptableLatency('in', to_ms(options.music_timestep))

proxy_out = nest.Create('music_event_out_proxy')
nest.SetStatus(proxy_out, {'port_name': 'out'})

parrot = nest.Create("parrot_neuron", NUM_ENC_NEURONS)

nest.Connect(proxy_in, parrot, 'one_to_one', {'delay': to_ms(options.music_timestep)})
for i in range(NUM_ENC_NEURONS):
    nest.Connect([parrot[i]], proxy_out, 'all_to_all', {'music_channel': i, 'delay': to_ms(options.music_timestep)})

comm.Barrier()
start = datetime.now()

nest.Simulate(to_ms(options.simtime))

end = datetime.now()
dt = end - start
run_time = dt.seconds + dt.microseconds / 1000000.

print 
print
print "RUN TIME:", run_time
print 
print

