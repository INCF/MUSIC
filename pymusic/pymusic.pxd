# The following include is needed to define a missing type MPI_Message
# which is lacking for a certain combination of MPI and mpi4py versions
cdef extern from "mpi_compat.h":
    pass

include "pyconfig.pxi"
cimport mpi4py.MPI as MPI
IF MPI4V2:
    from mpi4py.libmpi cimport *
ELSE:
    from mpi4py.mpi_c cimport *

from libcpp cimport bool as cbool
from libcpp.string cimport string
from libcpp.memory cimport unique_ptr, shared_ptr
from libcpp.vector cimport vector
from cpython.ref cimport PyObject

###########################################################

cdef extern from "music/predict_rank.hh" namespace "MUSIC":
    cdef int CPredictRank "MUSIC::predictRank" (int, char**)

cdef extern from "music/message.hh" namespace "MUSIC":
    cdef cppclass CMessageHandler "MUSIC::MessageHandler":
        pass

cdef extern from "music/music_c.h" namespace "MUSIC":
    cdef cppclass CEventHandler "MUSIC::EventHandler":
        pass
    cdef cppclass CEHLocal "MUSIC::EHLocal"(CEventHandler):
        CEHLocal(PyObject*)
    cdef cppclass CEHGlobal "MUSIC::EHGlobal"(CEventHandler):
        CEHGlobal(PyObject*)
    cdef cppclass CMHandler "MUSIC::MHandler"(CMessageHandler):
        CMHandler(PyObject*, cbool)

cdef extern from "music/event.hh" namespace "MUSIC":
    cdef cppclass CEventHandlerGlobalIndex "MUSIC::EventHandlerGlobalIndex":
        pass
    cdef cppclass CEventHandlerLocalIndex "MUSIC::EventHandlerLocalIndex":
        pass
    cdef cppclass CEventHandlerPtr "MUSIC::EventHandlerPtr":
        pass

cdef extern from "music/data_map.hh" namespace "MUSIC":
    cdef cppclass CDataMap "MUSIC::DataMap":
        pass

cdef extern from "music/index_map.hh" namespace "MUSIC":
    cdef cppclass CIndex "MUSIC::Index":
        int WILDCARD_MAX
    ctypedef enum IndexType "MUSIC::Index::Type":
            IndexGLOBAL "MUSIC::Index::GLOBAL",
            IndexLOCAL "MUSIC::Index::LOCAL"

    cdef cppclass GlobalIndex(CIndex):
        GlobalIndex(int)

    cdef cppclass LocalIndex(CIndex):
        LocalIndex(int)

    cdef cppclass CIndexMap "MUSIC::IndexMap":
        pass

cdef extern from "music/permutation_index.hh" namespace "MUSIC":
    cdef cppclass PermutationIndex(CIndexMap):
        PermutationIndex(GlobalIndex*, int)

cdef extern from "music/linear_index.hh" namespace "MUSIC":
    cdef cppclass LinearIndex(CIndexMap):
        LinearIndex(GlobalIndex, int)

cdef extern from "music/array_data.hh" namespace "MUSIC":
    cdef cppclass CArrayData "MUSIC::ArrayData"(CDataMap):
        CArrayData(void*, MPI_Datatype, int, int)
        CArrayData(void*, MPI_Datatype, CIndexMap*)

cdef extern from "music/port.hh" namespace "MUSIC":
    cdef cppclass CPort "MUSIC::Port":
        cbool isConnected()
        cbool hasWidth()
        string name()
        int width()

    cdef cppclass CContInputPort "MUSIC::ContInputPort"(CPort):
        pass

    cdef cppclass CContOutputPort "MUSIC::ContOutputPort"(CPort):
        pass

    cdef cppclass CEventInputPort "MUSIC::EventInputPort"(CPort):
        pass

    cdef cppclass CEventOutputPort "MUSIC::EventOutputPort"(CPort):
        pass

    cdef cppclass CMessageInputPort "MUSIC::MessageInputPort"(CPort):
        pass

    cdef cppclass CMessageOutputPort "MUSIC::MessageOutputPort"(CPort):
        void insertMessage(double, void*, size_t)

# This is necessary for the virtual downcast
cdef extern from *:
    CContInputPort*   dc_CContInputPort   \
        "dynamic_cast<MUSIC::ContInputPort*>" (CPort*)
    CContOutputPort*  dc_CContOutputPort  \
        "dynamic_cast<MUSIC::ContOutputPort*>"(CPort*)
    CEventInputPort*  dc_CEventInputPort  \
        "dynamic_cast<MUSIC::EventInputPort*>" (CPort*)
    CEventOutputPort* dc_CEventOutputPort \
        "dynamic_cast<MUSIC::EventOutputPort*>"(CPort*)
    CMessageInputPort*  dc_CMessageInputPort  \
        "dynamic_cast<MUSIC::MessageInputPort*>" (CPort*)
    CMessageOutputPort* dc_CMessageOutputPort \
        "dynamic_cast<MUSIC::MessageOutputPort*>"(CPort*)

cdef extern from *:
    shared_ptr[CPort] uc_shared_ptr_ContInputPort \
            "std::static_pointer_cast<MUSIC::Port>" (shared_ptr[CContInputPort])
    shared_ptr[CPort] uc_shared_ptr_ContOutputPort \
            "std::static_pointer_cast<MUSIC::Port>" (shared_ptr[CContOutputPort])
    shared_ptr[CPort] uc_shared_ptr_CEventInputPort \
            "std::static_pointer_cast<MUSIC::Port>" (shared_ptr[CEventInputPort])
    shared_ptr[CPort] uc_shared_ptr_CEventOutputPort \
            "std::static_pointer_cast<MUSIC::Port>" (shared_ptr[CEventOutputPort])
    shared_ptr[CPort] uc_shared_ptr_CMessageInputPort \
            "std::static_pointer_cast<MUSIC::Port>" (shared_ptr[CMessageInputPort])
    shared_ptr[CPort] uc_shared_ptr_CMessageOutputPort \
            "std::static_pointer_cast<MUSIC::Port>" (shared_ptr[CMessageOutputPort])

# cdef extern from "music/misc.hh" namespace "MUSIC":
#     ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)

# cdef extern from "music/connectivity.hh" namespace "MUSIC":
#     cdef cppclass CommunicationType:
#         pass
#     cdef cppclass ProcessingMethod:
#         pass

# cdef extern from "music/connectivity.hh" namespace "MUSIC::CommunicationType":
#     cdef CommunicationType COLLECTIVE
#     cdef CommunicationType POINTTOPOINT

# cdef extern from "music/connectivity.hh" namespace "MUSIC::ProcessingMethod":
#     cdef ProcessingMethod TREE
#     cdef ProcessingMethod TABLE

cdef extern from "music/connectivity.hh" namespace "MUSIC":
    ctypedef enum CommunicationType "MUSIC::CommunicationType":
        CommTypeCOLLECTIVE "MUSIC::CommunicationType::COLLECTIVE"
        CommTypePOINTTOPOINT "MUSIC::CommunicationType::POINTTOPOINT"

    ctypedef enum ProcessingMethod "MUSIC::ProcessingMethod":
        ProcMethodTREE "MUSIC::ProcessingMethod::TREE"
        ProcMethodTABLE "MUSIC::ProcessingMethod::TABLE"

    cdef extern from "music/configuration.hh" namespace "MUSIC":
        cdef cppclass CConfiguration "MUSIC::Configuration":
            pass

# TODO MusicContext
cdef extern from "music/port_manager.hh" namespace "MUSIC":
    cdef cppclass CPortConnectivityManager "MUSIC::PortConnectivityManager":
# TODO CommunicationType, ProcessingMethod
        void connect(string, string, string, string, int, CommunicationType,
                     ProcessingMethod)
        void disconnect(string, string)
        void disconnect(string, string, string, string)
        cbool isConnected(string)
        cbool isInstantiated(string)
        vector[shared_ptr[CPort]] getPorts()


cdef extern from "music/application.hh" namespace "MUSIC":
    cdef cppclass CApplication "MUSIC::Application":
        CApplication() except +
        CApplication(int&, char**&, double) except +
        CApplication(int&, char**&) except +
        CApplication(int&, char**&, int, int*, double) except +
        CApplication(int&, char**&, int, int*) except +

        cbool config(string, string*)

        shared_ptr[PortT] publish[PortT](string) except +

        void tick()
        void enterSimulationLoop(double)
        void exitSimulationLoop()
        void finalize()

        CPortConnectivityManager& getPortConnectivityManager()

        double time()
        double timebase()
        string name()

# cdef extern from "music/setup.hh" namespace "MUSIC":
#     cdef cppclass CSetup "MUSIC::Setup":
#         CSetup(int&, char**&) except +
#         CSetup(int&, char**&, int, int*) except +

#         cbool config(string, string*)

#         CContInputPort*     publishContInput(string)
#         CContOutputPort*    publishContOutput(string)
#         CEventInputPort*    publishEventInput(string)
#         CEventOutputPort*   publishEventOutput(string)
#         CMessageInputPort*  publishMessageInput(string)
#         CMessageOutputPort* publishMessageOutput(string)

# cdef extern from "music/runtime.hh" namespace "MUSIC":
#     cdef cppclass CRuntime "MUSIC::Runtime":
#         CRuntime(CSetup*, double) except +
#         void finalize()
#         double time()
#         void tick()

cdef extern void cython_callback(PyObject*, double, IndexType, int)

# TODO c-interface necessary?
cdef extern from "music/music_c.h" namespace "MUSIC":
    # cdef inline MPI_Comm communicator(CSetup*)
    # cdef inline MPI_Comm communicator(CRuntime*)
    cdef inline MPI_Comm communicator(CApplication*)
    cdef inline int toint(GlobalIndex)
    cdef inline int toint(LocalIndex)
    cdef inline cbool tick(CApplication*) except False
    cdef inline CEventHandlerPtr getEventHandlerPtr(IndexType, CEventHandler*)

    cdef inline void mapImpl "MUSIC::Implementer::mapImpl" (
        CContInputPort*, CDataMap*, double, int, cbool)
    cdef inline void mapImpl "MUSIC::Implementer::mapImpl" (
        CContOutputPort*, CDataMap*, int)
    cdef inline void mapImpl "MUSIC::Implementer::mapImpl" (
        CEventInputPort*, CIndexMap*, IndexType,
        CEventHandlerPtr, double, int)
    cdef inline void mapImpl "MUSIC::Implementer::mapImpl" (
        CEventOutputPort*, CIndexMap*, IndexType, int)
    cdef inline void insertEventImpl "MUSIC::Implementer::insertEventImpl" (
        CEventOutputPort*, double, int)
    cdef inline void mapImpl "MUSIC::Implementer::mapImpl" (
        CMessageInputPort*, CMessageHandler*, double, int)
    cdef inline void mapImpl "MUSIC::Implementer::mapImpl" (
        CMessageOutputPort*, int)
    cdef inline void insertMessage "MUSIC::Implementer::insertMessage" (
        CMessageOutputPort*, double t, void*, size)

    cdef cbool pythonError
    cdef PyObject* etype
    cdef PyObject* evalue
    cdef PyObject* etraceback


###########################################################
cdef class Application(object):
    cdef CApplication* ptr
    cdef list argv
    cdef int provided
    cdef readonly MPI.Intracomm comm
    cdef null(self)
    cpdef MPI.Intracomm getcomm(self)

###########################################################
cdef class PortManager(object):
    cdef CPortConnectivityManager* ref
    # cdef set ports

# cdef class Setup(object):
#     cdef CSetup* ptr
#     cdef list argv
#     cdef int provided
#     cdef readonly MPI.Intracomm comm
#     cdef set ports

#     cdef null(self)
#     cpdef MPI.Intracomm getcomm(self)

###########################################################

# cdef class Runtime(object):
#     cdef CRuntime* ptr
#     cdef readonly MPI.Intracomm comm
#     cdef set ports

###########################################################

cdef class Port(object):
    cdef shared_ptr[CPort] ptr
    # cpdef object null(self)

## Some day, virtual multiple inheritance will be
## a capital crime ^^^^

cdef class ContInputPort(Port):
    pass

cdef class ContOutputPort(Port):
    pass

cdef class EventInputPort(Port):
    cdef set events
    cpdef object null(self)

cdef class EventOutputPort(Port):
    pass

cdef class MessageInputPort(Port):
    cdef set events
    cpdef object null(self)

cdef class MessageOutputPort(Port):
    cdef bint pickled

##########################################################

from music.pybuffer cimport Buffer

cdef class DataMap(object):
    cdef CDataMap* ptr
    cdef Buffer buf

cdef class IndexMap(object):
    cdef CIndexMap* ptr
    cdef Buffer buf

cdef class EventHandler:
    cdef CEventHandler* ptr
    cdef object func

cdef class MessageHandler:
    cdef CMessageHandler* ptr
    cdef object func

cdef cbool EventCallback "MUSIC::EventCallback" ( \
  PyObject*, double, IndexType, int) except False

cdef cbool MessageCallback "MUSIC::MessageCallback" ( \
  PyObject*, double, void*, size_t, cbool) except False
