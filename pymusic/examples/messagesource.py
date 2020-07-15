#!/usr/bin/env python3

import music
from itertools import groupby, takewhile
from message import Msg

setup = music.Setup()
stoptime = setup.config("stoptime")
timestep = setup.config("timestep")
buf = setup.config("buffer")
events = setup.config("events")
pickled = setup.config("pickled") != 0

comm = setup.comm
rank = comm.Get_rank()
size = comm.Get_size()

out = setup.publishMessageOutput("out")
out.map(maxBuffered=buf, pickled=pickled)

eventgen = (i * stoptime / events
            for i in range(events)
            if i % size == rank)
steps = groupby(eventgen, lambda t: int(t/timestep))
def step(): return next(steps, (None, None))

runtime = setup.runtime(timestep)
times = takewhile(lambda t: t < stoptime, runtime)
nextStep, nextEvents = step()
for t in times:
    if int(t/timestep) != nextStep: continue

    for when in nextEvents:
        msg = Msg(rank, nextStep, t, when)
        if not pickled: msg = str(msg)
        out.insertMessage(when, msg)

    nextStep, nextEvents = step()

runtime.finalize()
