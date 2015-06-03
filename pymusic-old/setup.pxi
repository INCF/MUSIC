import sys

from setup cimport *

cdef class Setup:
    cdef cxx_Setup* cxx      # hold a C++ instance which we're wrapping
    def __cinit__(self, argv):
        # Convert argv into C array
        cdef int cxx_argc = 0
        cdef char* storage[100] #*fixme*
        cdef char** cxx_argv = storage
        for arg in argv:
            cxx_argv[cxx_argc] = arg
            cxx_argc += 1
            
        self.cxx = new_Setup (cxx_argc, cxx_argv)

        # Fill argv with C array
        argv[:] = []
        for i in range (0, cxx_argc):
            argv.append (cxx_argv[i])
            
    def __dealloc__(self):
        if self.cxx != NULL:
            del_Setup (self.cxx)

    def communicator (self):
        import music_late
        return wrapIntracomm (intracommToC (self.cxx.communicator ()))

    def publishEventOutput (self, identifier):
        return wrapEventOutputPort (self.cxx.publishEventOutput (identifier))

    def publishEventInput (self, identifier):
        return wrapEventInputPort (self.cxx.publishEventInput (identifier))

# Local Variables:
# mode: python
# End:
