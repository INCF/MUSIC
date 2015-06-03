from setup cimport *

cdef extern from "music/runtime.hh":
    ctypedef struct cxx_Runtime "MUSIC::Runtime":
        
        cxx_Intracomm communicator ()
        
        void tick ()
        
        double time ()
        
        void finalize ()
        
    cxx_Runtime *new_Runtime "new MUSIC::Runtime" (cxx_Setup* s, double h)
    
    void del_Runtime "delete" (cxx_Runtime *obj)

# Local Variables:
# mode: python
# End:
