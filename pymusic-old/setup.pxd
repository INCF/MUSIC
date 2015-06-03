#
#  This file is part of MUSIC.
#  Copyright (C) 2009 INCF
#
#  MUSIC is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
#
#  MUSIC is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

from port cimport *

from mpi4py.mpi_c cimport *

cdef extern from "late_impl.h":
    cdef wrapIntracomm (MPI_Comm comm)

cdef extern from "music/setup.hh":
    #
    # This is the Python binding of the Setup object in the MUSIC API
    #
    # It is documented in section 4.3 of the MUSIC manual
    #

    # From mpi2py (which we cannot load here):
    #ctypedef struct _mpi_comm_t
    #ctypedef _mpi_comm_t* MPI_Comm
    
    ctypedef struct cxx_Intracomm "MPI::Intracomm":
        MPI_Comm mpi_comm
    
    MPI_Comm intracommToC "(MPI_Comm)" (cxx_Intracomm comm)
    
    ctypedef struct cxx_Setup "MUSIC::Setup":

        cxx_Intracomm communicator ()

        #bool config (string var, string* result);

        #bool config (string var, int* result);

        #bool config (string var, double* result);

        #ContInputPort* publishContInput (string identifier);

        #ContOutputPort* publishContOutput (string identifier);

        cxx_EventInputPort* publishEventInput (char* identifier)

        cxx_EventOutputPort* publishEventOutput (char* identifier)

        #MessageInputPort* publishMessageInput (string identifier);

        #MessageOutputPort* publishMessageOutput (string identifier);

    cxx_Setup *new_Setup "new MUSIC::Setup" (int argc, char** argv)
    void del_Setup "delete" (cxx_Setup *obj)

#cdef extern object PyMPIComm_New (MPI_Comm)
    
# Local Variables:
# mode: python
# End:
