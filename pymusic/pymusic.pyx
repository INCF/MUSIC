#
# distutils: language = c++
# cython: c_string_encoding = default

from libc.stdlib cimport malloc, free

import mpi4py.MPI as MPI
import pickle

###########################################################

class MUSICError(Exception):
    """
    All exceptions in pymusic are MUSICError's
    """
    pass

class NoWidth(MUSICError):
    """
    Thrown if Port.width() is called, and port doesn't
    have a width defined.
    """
    def __init__(self):
        MUSICError.__init__(self, "No width defined")

class UndefinedConfig(MUSICError):
    """
    Thrown if a configuration value is requested
    via Setup.config(varname) for a variable name
    that has not been defined within the configuration.
    """
    def __init__(self, var):
        MUSICError.__init__(
            self,
            "Config variable {} is not defined".
            format(var)
        )

###########################################################

# """
# Just pass args as a named tuple for C functions that
# take the command line parameters:
#   int argc
#   char** argv
# argv = [exec, op[1], ..., op[argc-1], NULL]
# """
ctypedef struct Args:
    int argc
    char** argv

cdef Args argv_toc(list argv):
    """
    Convert an argv python list to a correct C argv & argc
    values with null termination, returned as an Args tuple
    """
    cdef Args r
    r.argc = len(argv)
    if r.argc <= 0:
        raise MUSICError("argv can't be empty")

    r.argv = <char**> malloc((r.argc+1) * sizeof(char*))
    if r.argv is NULL:
        raise MUSICError("couldn't allocate argv")

    cdef string
    try:
        r.argv[r.argc] = NULL
        for i in range(0, r.argc):
            s = argv[i].encode()
            r.argv[i] = s
        return r
    except:
        free(r.argv)
        raise

###########################################################

def predictRank(list argv=None):
    """
    Map into mpidep/predict_rank for config methods.
    AP: What is this really used for?
    """
    cdef Args r = argv_toc(argv if argv is not None else
                           sys.argv)
    return CPredictRank(r.argc, r.argv)

###########################################################

cdef class Port(object):
    """
    Base Port class.

    The underlying pointer is not deallocated here, since in MUSIC the
    port is owned by Setup/Runtime.  This does null the value when we
    believe the underlying object may be deallocated by the
    container.
    """
    def __cinit__(self):
        """
        When a Port object is created, it needs to have an underlying
        C++ class assigned to ptr from a Setup.publish* method.
        """
        self.ptr = NULL

    def __hash__(self):
        """
        Hash function: pointer value % to an int.
        """
        return <int><size_t> self.ptr

    cpdef null(self):
        """
        When Setup or Runtime no longer needs ports and so may deallocate
        them, we first must clear the references to them here. This
        may be overridden to give up references to other members by
        the descendants
        """
        self.ptr = NULL

    def isConnected(self):
        """
        return True iff the port is actually connected via the config
        file.
        """
        return self.ptr.isConnected()

    def width(self):
        """
        Returns the width set in the MUSIC config file
        which defines the number of indices transferred
        along this port. If no width is defined, raise a NoWidth
        exception.
        """
        if not self.ptr.hasWidth(): raise NoWidth()
        return self.ptr.width()

cdef class ContInputPort(Port):
    """
    ContInputPort maps data indices between sources and sinks. This is
    the sink side; source is ContOutputPort. Created by
    Setup.publishContInputPort(portname).
    """
    def map(self,
            object data,
            int base=0,
            object perm=None,
            double delay=0,
            cbool interpolate=True,
            int maxBuffered=-1):
        """
        map data as source for this port. (Can you do a multiple mapping?)

        object data: object to be shared that has a buffer interface.
                     That interface needs to be contiguous formed of
                     simple items that can be mapped to simple MPI
                     data types.
        int base (=0), object perm (=None): used to construct an
                     IndexMap (with number of items in data). See that
                     class for more information. Defines mapping for data
                     between this input source and the output sinks.
        double delay (=0): this is the delay of measurement relative
                           to "something" which I haven't been able to
                           figure out yet --- needs to be explained.
        interpolate (=True): whether to do a linear interpolation of
                             the values that are being delayed. Once
                             again, needs to be better explained.
        maxBuffered (=-1): buffer up to maxBuffered ticks. If -1, use
                           a "reasonable value" (???).  What does this
                           mean on the input side?
        """
        cdef Buffer buf = Buffer(data)
        cdef IndexMap imap = IndexMap(perm, base, buf.items)
        cdef DataMap d = DataMap(buf, imap)
        cdef CContInputPort* ptr = dc_CContInputPort(self.ptr)
        mapImpl(ptr, d.ptr, delay, maxBuffered, interpolate)

cdef class ContOutputPort(Port):
    """
    ContOutputPort maps data indices between sources and sinks. This
    is the source side; ContInputPort is the sink. Created by
    Setup.publishContOutputPort(portname).
    """
    def map(self, object data,
            int base=0, object perm=None,
            int maxBuffered=-1):
        """
        map data as sink for this port. (Can you do a multiple mapping?)

        object data: object to be shared that has a buffer interface.
                     That interface needs to be contiguous formed of
                     simple items that can be mapped to simple MPI
                     data types.
        int base (=0), object perm (=None): used to construct an
                     IndexMap (with number of items in data). See that
                     class for more information. Defines mapping for data
                     between this input source and the output sinks.
        maxBuffered (=-1): buffer up to maxBuffered ticks. If -1, use
                     a "reasonable value" (???).
        """
        cdef Buffer buf = Buffer(data)
        cdef IndexMap imap = IndexMap(perm, base, buf.items)
        cdef DataMap d = DataMap(buf, imap)
        cdef CContOutputPort* ptr = dc_CContOutputPort(self.ptr)
        mapImpl(ptr, d.ptr, maxBuffered)

cdef class EventInputPort(Port):
    """
    Event sink. Maps 'events' to handlers. Events are spike-like
    events at a defined time associated with an id, either/or
    global/local in some (unclear to me) way.

    Since these 'events' must be protected from deallocation as long
    as the port exists (until finalization), we keep a set of them
    here and drop the reference when this object is null'd (events member).
    """
    def __cinit__(self):
        """
        Store a set of references to the event handlers
        """
        self.events = set()

    cpdef null(self):
        """
        Not just null pointer, but give up the associated event
        handlers.
        """
        Port.null(self)
        self.events = None

    def map(self, func, IndexType t,
            double accLatency=0, int maxBuffered=-1,
            object perm=None, int base=-1, int size=-1):
        """
        Map events to this sink.
        func is:
          void func(double value, IndexType GLOBAL|LOCAL, int index)
        IndexType is (?) the index type to map the incoming sources
            passed to the func (why, if we already know this?)
        accLatency is the delivery delay relative to calculation time
            (? is it ? What formula are we using here?)
        maxBuffered is ticks to buffer
        perm, base & size are passed to IndexMap
        """
        cdef IndexMap imap = IndexMap(perm, base, size)
        cdef EventHandler eh = EventHandler(func, t)
        cdef CEventInputPort* ptr = dc_CEventInputPort(self.ptr)
        cdef CEventHandlerPtr hndl = getEventHandlerPtr(t, eh.ptr)
        mapImpl(ptr, imap.ptr, t, hndl, accLatency, maxBuffered)
        self.events.add(eh)

cdef class EventOutputPort(Port):
    """
    Map events from this sink. See EventInputPort for description of
    events.
    """
    def map(self, IndexType t, int maxBuffered=-1,
            object perm=None, int base=-1, int size=-1):
        """
        IndexType: LOCAL/GLOBAL
        maxBuffered: tick to buffer (why on both sides?)
        perm, base, size: define IndexMap (see that class)
        """
        cdef IndexMap m = IndexMap(perm, base, size)
        cdef CEventOutputPort* ptr = dc_CEventOutputPort(self.ptr)
        mapImpl(ptr, m.ptr, t, maxBuffered)

    def insertEvent(self, double time, int index, IndexType t):
        """
        double time: time of event (must be during current tick)
        int index: local or global index of event
        IndexType t: LOCAL|GLOBAL
        """
        cdef CEventOutputPort* ptr = dc_CEventOutputPort(self.ptr)
        insertEventImpl(ptr, time, index)

cdef class MessageInputPort(Port):
    """
    Maps 'events' to handlers.

    Since these 'events' must be protected from deallocation as long
    as the port exists (until finalization), we keep a set of them
    here and drop the reference when this object is null'd.
    """
    def __cinit__(self):
        self.events = set()

    cpdef null(self):
        """
        Remember to remove the references to events...
        """
        Port.null(self)
        self.events = None

    def map(self, func, double accLatency=0, int maxBuffered=-1, bint pickled=True):
        """
        The message handling function.
        func: callable object as follows
           void func(double time, object msg):
             time: the set insertion time of the message
             msg: an object which may have been pickled and fed through music
        accLatency(0): "acceptable latency" (???)
        maxBuffered(-1): ticks that can be buffered (-1 means unknown
           amount (???))
        pickled(True): pickle objects on both ends; else, the object should be
                       a buffer object and we'll receive a bytearray
        """
        cdef MessageHandler eh = MessageHandler(func, pickled)
        cdef CMessageInputPort* ptr = dc_CMessageInputPort(self.ptr)
        mapImpl(ptr, eh.ptr, accLatency, maxBuffered)
        self.events.add(eh)

cdef class MessageOutputPort(Port):
    """
    The source of "messages". A message is a python object that is
    pickled and fed to the other end, associated with a time.
    """
    def map(self, int maxBuffered=-1, bint pickled=True):
        """
        map only sets the buffering ticks.
        maxBuffered (-1): ticks to buffer (-1 means "reasonable" ???)
        pickled (True): whether messages objects inserted her are automatically
                        pickled, else the object should be a buffer-type and
                        the other end will get a bytearray.
        """
        self.pickled = pickled
        cdef CMessageOutputPort* ptr = dc_CMessageOutputPort(self.ptr)
        mapImpl(ptr, maxBuffered)

    def insertMessage(self, double time, object msg):
        """
        insert a python object as a message at a time during the
        current tick.
        time: ms times (??)
        msg: a python object that can be cPickled if self.pickled == True
             else something that has a buffer interface.
        """
        cdef CMessageOutputPort* ptr = dc_CMessageOutputPort(self.ptr)
        cdef bytearray pmsg =                                           \
                <bytearray> pickle.dumps(msg, pickle.HIGHEST_PROTOCOL)  \
                if self.pickled                                         \
                else <bytearray> msg
        ptr.insertMessage(time, <char*> pmsg, len(pmsg))

###########################################################

import sys
cdef class Setup(object):
    """
    API to setup the music interface.
    Becomes invalid after a Runtime object is created.
    It gives access to config variables (*.music file),
    and creates sinks and sources which can then be configured with
    the respective map method of each.
    """

    def __cinit__(self, list argv=None, required=None):
        """
        Takes the command line arguments and the required threading
        level.
        string[] argv (None): comand line args; if None, use sys.argv
        int required (None): the MPI threading level for MPI-2. If
        None, don't set at all, but provided level in object set to
        MPI_THREAD_SINGLE. Also stores command line arguments.
        Calls CSetup object and connects with communicator.
        """

        cdef int argc
        cdef char** argv_c
        cdef int provided

        cdef Args r = argv_toc(argv if argv is not None
                               else sys.argv)
        try:
            if required is None:
                self.ptr = new CSetup(r.argc, r.argv)
                self.provided = MPI_THREAD_SINGLE
            else:
                self.ptr = new CSetup(r.argc, r.argv, required, &provided)
                self.provided = provided
            self.argv = [r.argv[i] for i in xrange(r.argc)]
        finally:
            free(r.argv)

        cdef MPI.Intracomm comm = MPI.Intracomm()
        comm.ob_mpi = communicator(self.ptr)
        self.comm = comm

        self.ports = set()

    def __dealloc__(self):
        """
        If runtime has been made and setup is invalid, do nothing.
        Otherwise, deallocate or remove references to all objects.
        """

        if self.ptr is NULL:
            return

        del self.ptr
        for p in self.ports:
            del p.ptr
            p.null()
        self.null()

    cpdef MPI.Intracomm getcomm(self):
        """
        Get an MPI communicator between the processes on this current
        side of music
        """
        return self.comm

    cdef null(self):
        """
        Invalidate this object --- called when runtime is
        created. Music & runtime is now responsible for deallocating C
        objects and clearing references to them.
        """
        self.ptr = NULL
        self.comm = <MPI.Intracomm> MPI.COMM_NULL
        self.ports = None

    ####
    def config(self, string var):
        """
        Get the value for var in config file.
        string var: configuration variable for current group of
        processes, including default and inherited settings.
        The value is returned as an int if it matches the patter,
        otherwise a float, otherwise a string.
        Throws UndefinedConfig error if no such variable is defined.
        """
        cdef string vs

        if not self.ptr.config(var, &vs):
            raise UndefinedConfig(var)

        try: return int(vs)
        except ValueError: pass

        try: return float(vs)
        except ValueError: pass

        return vs
    ####

    def publishContInput(self, string s):
        """
        Continuous Floating point value sink
        string s: port name for music config
        """
        cdef ContInputPort p = ContInputPort()
        p.ptr = self.ptr.publishContInput(s) # can't pass pointers directly to init
        self.ports.add(p)
        return p

    def publishContOutput(self, string s):
        """
        Continuous Floating point value source
        string s: port name for music config
        """
        cdef ContOutputPort p = ContOutputPort()
        p.ptr = self.ptr.publishContOutput(s)
        self.ports.add(p)
        return p

    def publishEventOutput(self, string s):
        """
        Discrete boolean source
        string s: port name for music config
        """
        cdef EventOutputPort p = EventOutputPort()
        p.ptr = self.ptr.publishEventOutput(s)
        self.ports.add(p)
        return p

    def publishEventInput(self, string s):
        """
        Discrete boolean sink
        string s: port name for music config
        """
        cdef EventInputPort p = EventInputPort()
        p.ptr = self.ptr.publishEventInput(s)
        self.ports.add(p)
        return p

    def publishMessageOutput(self, string s):
        """
        Discrete python object source
        string s: port name for music config
        """
        cdef MessageOutputPort p = MessageOutputPort()
        p.ptr = self.ptr.publishMessageOutput(s)
        self.ports.add(p)
        return p

    def publishMessageInput(self, string s):
        """
        Discrete python object sink
        string s: port name for music config
        """
        cdef MessageInputPort p = MessageInputPort()
        p.ptr = self.ptr.publishMessageInput(s)
        self.ports.add(p)
        return p

    def runtime(self, double timestep):
        """
        Create a runtime.
        After return, this Setup object is invalid
        double timestep: size of timestep (?? units)
        """
        return Runtime(self, timestep)

###########################################################

cdef class Runtime(object):
    def __cinit__(self, Setup setup, double h):
        self.ptr = new CRuntime(setup.ptr, h)
        self.ports = setup.ports
        self.isFinalized = 0
        setup.null()

        cdef MPI.Intracomm comm = MPI.Intracomm()
        comm.ob_mpi = communicator(self.ptr)
        self.comm = comm

    def __dealloc__(self):
        if not self.isFinalized:
           self.ptr.finalize()

        for p in self.ports:
            p.null()

        del self.ptr

    def time(self): return self.ptr.time()
    
    def tick(self): tick(self.ptr)

    def finalize(self):
        self.isFinalized = 1
        self.ptr.finalize()

    def __iter__(self):
        cdef CRuntime* ptr = self.ptr
        while True:
            yield ptr.time()
            tick(ptr)

###########################################################

from music.pybuffer import Buffer

from cpython cimport array
from array import array

cdef class IndexMap:
    """
    Internal:
    Contains a C++ LinearIndex or PermutationIndex.
    These map between "local" index i to "global" index j
    LinearIndex: g = l + base, for l < size
    PermutationIndex: g = perm[l], where perm is an array
    """
    def __cinit__(self, object perm=None, int base=0, int size=-1):
        """
        Created internally
        Either perm is None, or we have a base, size pair
        perm is buffer of integers mapping local to global
          (local[i] -> global[perm[i]])
        otherwise base is the first global index for size elements
          (local[0] -> global[base])
        """
        cdef array.array arr

        if perm is None:
            self.buf = None
            self.ptr = new LinearIndex(GlobalIndex(base), size)
        else:
            self.buf = Buffer(perm)
            if self.buf.dtype.size() != sizeof(int):
                arr = array('i', perm)
                self.buf = Buffer(arr)
            self.ptr = new PermutationIndex(<GlobalIndex*>
                                            self.buf.pybuf.buf,
                                            self.buf.items)

    def __dealloc__(self):
        del self.ptr

cdef class DataMap(object):
    """
    Internal:
    An ArrayData object. Takes a data object as a Buffer and possibly an
    IndexMap to construct a mapping for the Cont*Port transfers.
    """
    def __cinit__(self, Buffer buf, IndexMap index_map=None, int index=0):
        """
        Maps the Buffer of a data object between local and global indices.
        Buffer buf: some simple array type that will be pushed through mpi
        IndexMap index_map (None): an option index mapping from local to global
        index (0): if index_map is None, this is the base offset for constructing
                   a linear index
        """
        self.buf = buf
        self.ptr = new CArrayData(buf.pybuf.buf,
                                  buf.dtype.ob_mpi,
                                  index, buf.items) \
                   if index_map is None else \
                   new CArrayData(buf.pybuf.buf,
                                  buf.dtype.ob_mpi,
                                  index_map.ptr)

    def __dealloc__(self): del self.ptr

cdef class EventHandler:
    """
    Internal:
    An EventHandler is the python wrapper around a global
    or local EventHandler object for EventInputPort
    """
    def __cinit__(self, object func, IndexType t):
        """
        func: a callable of the form
          func(double d, IndexType t, int i) where 
             double d: the event time
             IndexType t: local or global index enum
             int i: the index value
        IndexType t: the IndexType to receive in func
             (??? Why is this so complex?? Do we need
              to generalize func like this??)
        """
        self.ptr = <CEventHandler*>                \
                   new CEHGlobal(<PyObject*>func) \
                   if t == IndexGLOBAL else       \
                   <CEventHandler*>               \
                   new CEHLocal(<PyObject*>func)
        self.func = func

    def __dealloc__(self): del self.ptr

cdef class _Index:
    """
    Internal: the type of the variable Index
    that encapsulates the enum mapping
    GLOBAL and LOCAL are the ints of the C enums
    backmap maps from the enum to a string for output
    purposes
    """
    cdef readonly int GLOBAL
    cdef readonly int LOCAL
    cdef readonly dict backmap

    def __cinit__(self):
        """ SINGLETON """
        self.GLOBAL = IndexGLOBAL
        self.LOCAL = IndexLOCAL
        self.backmap = {
            IndexGLOBAL: "GLOBAL",
            IndexLOCAL: "LOCAL"
        };

    def tostr(self, int index):
        """" tostring(Index.{GLOBAL,LOCAL}) --> "{GLOBAL,LOCAL}" """
        return self.backmap[index]

# And here is the singleton def
Index = _Index()

cdef cbool EventCallback(PyObject* func,
                         double d,
                         IndexType t,
                         int i) \
    except False:
    """
    Internal: C code can map back into python through this callback
    func: callable object of func(d, t, i)
    double d: time of event
    IndexType t: LOCAL/GLOBAL int
    int i: index of event
    """
    (<object>func)(d, t, i)
    return True

cdef class MessageHandler:
    """
    MessageHandler wraps up Message*Port communications between C & python,
    wrapping n particular the C++ MessageHandler object bound with a function.
    It may (or may not) depickle messages
    """
    def __cinit__(self, object func, bint pickled):
        """
        func: The function to be called on a message, of the form
          func(double t, object msg) where:
            double t: the message time
            msg: Either a bytearray, or an arbitrary unpickled python object
        pickled: if true, the msg needs to be depickled
        """
        self.ptr = new CMHandler(<PyObject*>func, pickled)
        self.func = func

    def __dealloc__(self):  del self.ptr

cdef cbool MessageCallback(PyObject* func,
                           double t,
                           void* msg,
                           size_t s,
                           cbool pickled) \
    except False:
    """
    The function to return a Message from C to Python.
    func: callable of the form
       func(double t, object/bytearray obj) where:
         t: is the time of the message
         obj: is either the unpickled object if the port unpickles
              or simply a bytearray of data (you figure out the source)
    """
    #cdef str pobj = (<char*>msg)[:s]
    cdef bytearray pobj = (<char*>msg)[:s]
    cdef object obj = pickle.loads(pobj) if pickled else pobj
    (<object>func)(t, obj)
    return True

#############################################################################
#
# And for handling errors at the C/Python interface
#
# pythonError: true if a python error is being stored until we get out of C
# etype, evalue, etraceback: three bits of data to recreate the exception 
#    so that it can be thrown
#
pythonError = False
etype = NULL
evalue = NULL
etraceback = NULL
