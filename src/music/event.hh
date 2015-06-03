/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009 INCF
 *
 *  MUSIC is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  MUSIC is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MUSIC_EVENT_HH
#include <music/index_map.hh>

namespace MUSIC {

  class Event {
  public:
    double t;
    int id;
    Event (double t_, int id_) : t (t_), id (id_) { }
    bool operator< (const Event& other) const { return t < other.t; }
  };


  class EventHandlerGlobalIndex {
  public:
    virtual void operator () (double t, GlobalIndex id) = 0;
  };
  
  class EventHandlerGlobalIndexDummy : public EventHandlerGlobalIndex {
  public:
    virtual void operator () (double t, GlobalIndex id) { };
  };
  
  class EventHandlerGlobalIndexProxy
    : public EventHandlerGlobalIndex {
    void (*eventHandler) (double t, int id);
  public:
    EventHandlerGlobalIndexProxy () { }
    EventHandlerGlobalIndexProxy (void (*eh) (double t, int id))
      : eventHandler (eh) { }
    void operator () (double t, GlobalIndex id)
    {
      eventHandler (t, id);
    }
  };
  
  class EventHandlerLocalIndex {
  public:
    virtual void operator () (double t, LocalIndex id) = 0;
  };

  class EventHandlerLocalIndexDummy : public EventHandlerLocalIndex {
  public:
    virtual void operator () (double t, LocalIndex id) { };
  };

  class EventHandlerLocalIndexProxy
    : public EventHandlerLocalIndex {
    void (*eventHandler) (double t, int id);
  public:
    EventHandlerLocalIndexProxy () { }
    EventHandlerLocalIndexProxy (void (*eh) (double t, int id))
      : eventHandler (eh) { }
    void operator () (double t, LocalIndex id)
    {
      eventHandler (t, id);
    }
  };

  class EventHandlerPtr {
    union {
      EventHandlerGlobalIndex* global;
      EventHandlerLocalIndex* local;
    } ptr;
    Index::Type type_;
  public:
    EventHandlerPtr () { }
    EventHandlerPtr (EventHandlerGlobalIndex* p):type_(Index::GLOBAL) { ptr.global = p; }
    EventHandlerPtr (EventHandlerLocalIndex* p):type_(Index::LOCAL) { ptr.local = p; }
    typedef EventHandlerGlobalIndex* EventHandlerGlobalIndexPtr;
    operator EventHandlerGlobalIndex* () { return ptr.global; }
    operator EventHandlerLocalIndex* () { return ptr.local; }
    Index::Type getType(){return type_;}
  };
  
}
#define MUSIC_EVENT_HH
#endif
