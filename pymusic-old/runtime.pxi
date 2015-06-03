from runtime cimport *

cdef class Runtime:
    cdef cxx_Runtime *cxx      # hold a C++ instance which we're wrapping
    def __cinit__(self, Setup setup, h):
        self.cxx = new_Runtime (setup.cxx, h)
        # setup object is now deallocated
        setup.cxx = NULL       # mark as deallocated

    def __dealloc__(self):
        del_Runtime (self.cxx)
        
    def communicator (self):
        import music_late
        return wrapIntracomm (intracommToC (self.cxx.communicator ()))

    def tick (self):
        self.cxx.tick ()

    def time (self):
        return self.cxx.time ()

    def finalize (self):
        self.cxx.finalize ()

# Local Variables:
# mode: python
# End:
