#!/usr/bin/env python

import music
from itertools import groupby, takewhile
import sys

setup = music.Setup()
stoptime = setup.config("stoptime")
timestep = setup.config("timestep")
buf = setup.config("buffer")
events = setup.config("events")

comm = setup.comm
rank = comm.Get_rank()
size = comm.Get_size()

out = setup.publishEventOutput("out")

width = out.width()
local = width // size
rest = width % size
firstId = rank * local

# distribute the rest:
firstId += min(rank, rest)
if rank < rest: local += 1

index = music.Index.LOCAL

if index == music.Index.LOCAL:
    def mapout(i): return i
else:
    def mapout(i): return i + firstId

out.map(index,
        base=firstId,
        size=local,
        maxBuffered=buf)

eventgen = ((mapout(i % local), i * stoptime / events)
            for i in range(events)) # index, time
steps = groupby(eventgen, lambda i_t: int(i_t[1]/timestep))
def step(): return next(steps, (None, None))

runtime = setup.runtime(timestep)
times = takewhile(lambda t: t < stoptime, runtime)
nextStep, nextEvents = step()
for t in times:
    if int(t/timestep) != nextStep: continue

    for i, when in nextEvents:
        sys.stderr.write("Insert rank {}: val={}, type={}, ind={}\n".
                         format(rank, when, i, music.Index.tostr(index)))
        out.insertEvent(when, i, index)

    nextStep, nextEvents = step()
