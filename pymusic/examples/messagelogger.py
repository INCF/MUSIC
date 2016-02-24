#!/usr/bin/env python

import music
import sys
from itertools import takewhile

setup = music.Setup()
stoptime = setup.config("stoptime")
timestep = setup.config("timestep")
buf = setup.config("buffer")
pickled = setup.config("pickled") != 0

errorAt = setup.config("errorAt")
if errorAt < 0: errorAt = None

comm = setup.comm
rank = comm.Get_rank()
size = comm.Get_size()

def eventerr(d):
    if errorAt is None: return
    if d >= errorAt: raise RuntimeError("Hey")

time = None
def msgfunc(d, msg):
    print (msg)
    eventerr(d)
    sys.stderr.write(
        "Receive rank {}: {} ({}) at {}\n".
        format(rank, type(msg), msg, d))

inp = setup.publishMessageInput("in")
inp.map(msgfunc, maxBuffered=buf, pickled=pickled)

runtime = setup.runtime(timestep)
times = takewhile(lambda t: t <= stoptime, runtime)
for time in times:
    pass
