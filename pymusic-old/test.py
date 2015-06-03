#!/usr/local/bin/nrniv -python

import sys
import music

s = music.Setup (sys.argv)
p = s.publishEventOutput ("out")
print p
