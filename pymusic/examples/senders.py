#! /usr/bin/env python3

import music

import sys
import numpy
from itertools import takewhile

setup = music.Setup()
stoptime = setup.config("stoptime")
timestep = setup.config("timestep")

comm = setup.comm
rank = comm.Get_rank()

out = setup.publishContOutput("out")
data = numpy.array([-1], dtype=numpy.int)
out.map(data, base=rank)

runtime = setup.runtime(timestep)
times = takewhile(lambda t: t < stoptime, runtime)
for time in times:
    data[0] = rank
    sys.stdout.write("t={}\tsender {}: Hello!\n".format(time, rank))

runtime.finalize()
