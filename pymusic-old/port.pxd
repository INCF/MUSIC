import sys

cdef extern from "music/port.hh":
    ctypedef struct cxx_EventOutputPort "MUSIC::EventOutputPort":
        pass
    ctypedef struct cxx_EventInputPort "MUSIC::EventInputPort":
        pass

# Local Variables:
# mode: python
# End:
