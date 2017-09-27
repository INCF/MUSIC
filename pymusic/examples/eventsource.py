#!/usr/bin/env python

import music
from itertools import groupby, takewhile
import sys

application = music.Application()
stoptime = application.config("stoptime")
timestep = application.config("timestep")
buf = application.config("buffer")
events = application.config("events")

comm = application.comm
rank = comm.Get_rank()
size = comm.Get_size()

out = application.publishEventOutput("out")

width = out.width()
local = width // size
rest = width % size
firstId = rank * local

# distribute the rest:
firstId += min(rank, rest)
if rank < rest: local += 1

out.map(music.Index.GLOBAL,
        base=firstId,
        size=local,
        maxBuffered=buf)

eventgen = ((firstId + i % local, i * stoptime / events)
            for i in range(events)) # index, time
steps = groupby(eventgen, lambda i_t: int(i_t[0]/timestep))
def step(): return next(steps, (None, None))

# runtime = application.runtime(timestep)
application.enterSimulationLoop(timestep)
times = takewhile(lambda t: t < stoptime, application)
nextStep, nextEvents = step()
for t in times:
    if int(t/timestep) != nextStep: continue

    for index, when in nextEvents:
        sys.stderr.write("Insert rank {}: Event ({}, {}) at {}\n".
                         format(rank, index, when, t))
        out.insertEvent(when, index, music.Index.GLOBAL)

    nextStep, nextEvents = step()
application.finalize()
