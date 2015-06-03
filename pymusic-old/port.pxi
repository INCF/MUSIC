import sys

from port cimport *

cdef class EventOutputPort:
    cdef cxx_EventOutputPort* cxx
    def __cinit__(self):
        pass

cdef wrapEventOutputPort (cxx_EventOutputPort* port):
    cdef EventOutputPort port_ = EventOutputPort ()
    port_.cxx = port
    return port_

cdef class EventInputPort:
    cdef cxx_EventInputPort* cxx
    def __cinit__(self):
        pass

cdef wrapEventInputPort (cxx_EventInputPort* port):
    cdef EventInputPort port_ = EventInputPort ()
    port_.cxx = port
    return port_

# Local Variables:
# mode: python
# End:
