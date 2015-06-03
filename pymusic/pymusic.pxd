cimport mpi4py.MPI as MPI
from mpi4py.mpi_c cimport *

from libcpp cimport bool as cbool
from libcpp.string cimport string
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

cdef extern from "music/setup.hh" namespace "MUSIC":
    cdef cppclass CSetup "MUSIC::Setup":
        CSetup(int&, char**&) except +
        CSetup(int&, char**&, int, int*) except +

        cbool config(string, string*)

        CContInputPort*     publishContInput(string)
        CContOutputPort*    publishContOutput(string)
        CEventInputPort*    publishEventInput(string)
        CEventOutputPort*   publishEventOutput(string)
        CMessageInputPort*  publishMessageInput(string)
        CMessageOutputPort* publishMessageOutput(string)

cdef extern from "music/runtime.hh" namespace "MUSIC":
    cdef cppclass CRuntime "MUSIC::Runtime":
        CRuntime(CSetup*, double) except +
        void finalize()
        double time()
        void tick()

cdef extern void cython_callback(PyObject*, double, IndexType, int)

cdef extern from "music/music_c.h" namespace "MUSIC":
    cdef inline MPI_Comm communicator(CSetup*)
    cdef inline MPI_Comm communicator(CRuntime*)
    cdef inline int toint(GlobalIndex)
    cdef inline int toint(LocalIndex)
    cdef inline cbool tick(CRuntime*) except False
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

cdef class Setup(object):
    cdef CSetup* ptr
    cdef list argv
    cdef int provided
    cdef readonly MPI.Intracomm comm
    cdef set ports

    cdef null(self)
    cpdef MPI.Intracomm getcomm(self)

###########################################################

cdef class Runtime(object):
    cdef CRuntime* ptr
    cdef readonly MPI.Intracomm comm
    cdef set ports

###########################################################

cdef class Port(object):
    cdef CPort* ptr
    cpdef object null(self)

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
