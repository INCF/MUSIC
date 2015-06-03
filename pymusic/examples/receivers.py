#! /usr/bin/python

import music

import sys
import numpy
from itertools import takewhile, dropwhile

setup = music.Setup()
stoptime = setup.config("stoptime")
timestep = setup.config("timestep")

comm = setup.comm
rank = comm.Get_rank()

pin = setup.publishContInput("in")
data = numpy.array([-2], dtype=numpy.int)
pin.map(data, base=rank)

runtime = setup.runtime(timestep)
mintime = timestep
maxtime = stoptime+timestep
start = dropwhile(lambda t: t < mintime, runtime)
times = takewhile(lambda t: t < maxtime, start)
for time in times:
    srank = data[0]
    sys.stdout.write(
        "t={}\treceiver {}: received Hello from sender {}\n".
        format(time, rank, srank))
