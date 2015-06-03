#!/usr/bin/env python
"""
MUSIC Hello World
"""

import sys, music
setup = music.Setup (sys.argv)
from mpi4py import MPI

hwmess = "Hello, World! I am process %d of %d with argument %s.\n"
comm = setup.communicator ()
myrank = comm.Get_rank()
nprocs = comm.Get_size()
sys.stdout.write(hwmess % (myrank, nprocs, sys.argv[1]))

# Entering runtime phase
runtime = music.Runtime (setup, 1e-4)

# Must end with a call to finalize ()
runtime.finalize ()
