import os

from music import *

define ('stoptime', 1.0)

app1 = Application (2, 'eventsource', '-b 1 10 spikes')
app2 = Application (2, 'eventlogger', '-b 2')

connect (app1, 'out', app2, 'in', 10)

launch ()
