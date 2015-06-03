#ifndef MUSIC_C_H
#define MUSIC_C_H

#include <Python.h>

#include <mpi.h>
#include "music/setup.hh"
#include "music/runtime.hh"
#include "music/index_map.hh"
#include "music/event.hh"
#include "music/message.hh"
#include "music/configuration.hh"

#include <iostream>
#include <string>

namespace MUSIC {
  using namespace std;

  inline MPI_Comm communicator(MUSIC::Setup* s) {
    return (MPI_Comm) s->communicator();
  }

  inline MPI_Comm communicator(MUSIC::Runtime* r) {
    return (MPI_Comm) r->communicator();
  }

  inline int toint(MUSIC::GlobalIndex i) {
    return i;
  }

  inline int toint(MUSIC::LocalIndex i) {
    return i;
  }

  static bool pythonError;
  static PyObject* etype;
  static PyObject* evalue;
  static PyObject* etraceback;

  bool tick(Runtime* ptr) {
    ptr->tick();
    if (! pythonError) return true;

    pythonError = false;
    PyErr_Restore(etype, evalue, etraceback);
    return false;
  }

  bool EventCallback(PyObject* func,
		     double d,
		     Index::Type t,
		     int index);

  class EventHandler { // All this just to insert a virtual d
  public:
    PyObject* const func;

    EventHandler(PyObject* func): func(func) {}
    virtual ~EventHandler() {}
    inline void callback(double d, Index::Type t, int i) {
      if (pythonError) return;
      if (EventCallback(func, d, t, i)) return;

      pythonError = true;
      PyErr_Fetch(&etype, &evalue, &etraceback);
    }
  };


  class EHLocal: public EventHandler,
		 public EventHandlerLocalIndex {
  public:
    EHLocal(PyObject* func): EventHandler(func) {};
    void operator() (double d, LocalIndex i) {
      callback(d, Index::LOCAL, i);
    }
  };

  class EHGlobal: public EventHandler, public EventHandlerGlobalIndex {
  public:
    EHGlobal(PyObject* func): EventHandler(func) {};
    void operator() (double d, GlobalIndex i) {
      callback(d, Index::GLOBAL, i);
    }
  };

  inline
  EventHandlerPtr getEventHandlerPtr(Index::Type t,
				     EventHandler* eh) {
    return (t == Index::GLOBAL)
      ? EventHandlerPtr((EHGlobal*) eh)
      : EventHandlerPtr((EHLocal*) eh);
  }

  bool MessageCallback(PyObject* func,
		       double t,
		       void* msg,
		       size_t size,
		       bool pickled);

  class MHandler: public MessageHandler {
  public:
    PyObject* const func;
    const bool pickled;

    MHandler(PyObject* func, bool pickled):
      func(func), pickled(pickled) {}
    void operator () (double t, void* msg, size_t size) {
      if (pythonError) return;
      if (MessageCallback(func, t, msg, size, pickled)) return;

      pythonError = true;
      PyErr_Fetch(&etype, &evalue, &etraceback);
    }
  };

  class Implementer {
  public:
    static inline
    void mapImpl(ContInputPort* p,
		 DataMap* d,
		 double v,
		 int i,
		 bool b) {
      p->ContInputPort::mapImpl(d, v, i, b);
    }

    static inline
    void mapImpl(ContOutputPort* p,
		 DataMap* d,
		 int i) {
      p->ContOutputPort::mapImpl(d, i);
    }

    static inline
    void mapImpl(EventInputPort* p,
		 IndexMap* m,
		 Index::Type t,
		 EventHandlerPtr e,
		 double d,
		 int i) {
      p->EventInputPort::mapImpl(m, t, e, d, i);
    }

    static inline
    void mapImpl(EventOutputPort* p,
		 IndexMap* m,
		 Index::Type t,
		 int i) {
      p->EventOutputPort::mapImpl(m, t, i);
    }

    static inline
    void insertEventImpl(EventOutputPort* p,
			 double d,
			 int i) {
      p->EventOutputPort::insertEventImpl(d, i);
    }
    static inline
    void mapImpl(MessageInputPort* p,
		 MessageHandler* handler,
		 double accLatency,
		 int maxBuffered) {
      p->MessageInputPort::mapImpl(handler,
				   accLatency,
				   maxBuffered);
    }

    static inline
    void mapImpl(MessageOutputPort* p,
		 int maxBuffered) {
      p->MessageOutputPort::mapImpl(maxBuffered);
    }
  };
}

#endif // MUSIC_C_H
