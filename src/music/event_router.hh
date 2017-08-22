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

#ifndef MUSIC_EVENT_ROUTER_HH
#include <map>
#include <vector>

#include <music/music-config.hh>
#include <music/FIBO.hh>
#include <music/index_map.hh>
#include <music/event.hh>

#include <music/interval_tree.hh>

#define MUSIC_ITABLE_FLAVOR MUSIC_SCATTERED
//#define MUSIC_ITABLE_COMPRESSED
#include <music/interval_table.hh>

namespace MUSIC {
  /* remedius
   * Since the router can act as on the output (point-to-point communication)
   * as well as on the input (collective communication) sides and
   * pre-/post-processing approaches are different
   * (in one case it's insertion to the buffer <FIBO>,
   * in another case it's calling appropriate handler <EventHandlerGlobalIndex>),
   * two more successors for EventRoutingData were created:
   * InputRoutingData and OutputRoutingData.
   */

  class EventRoutingData {
    int offset_;
  public:
    EventRoutingData () { }
    EventRoutingData (const IndexInterval &i) : offset_ (i.local ()) { }
    EventRoutingData (const EventRoutingData& e) : offset_ (e.offset_) { }
    ~EventRoutingData(){};
    int offset () const { return offset_; }
  };

  template<class EventHandler>
  class InputRoutingData : public EventRoutingData {
    EventHandler* eventHandler_;
  public:
    InputRoutingData (const IndexInterval &i, EventHandlerPtr* h)
      : EventRoutingData (i), eventHandler_ (*h) { }
    InputRoutingData () : eventHandler_ (NULL) { }
    void process (double t, int id) { (*eventHandler_) (t, id); }
  };

  class OutputRoutingData : public EventRoutingData {
    FIBO* buffer_;
  public:
    OutputRoutingData (const IndexInterval &i, FIBO* b);
    OutputRoutingData () {}
    void process (double t, int id);
  };

  class DirectRouter; //Remove this later

  /* remedius
   * We've decided to try different methods for pre-/post-processing the data:
   * using tree algorithm and using table.
   * Hence to this two EventRouter successors were created:
   * TableProcessingRouter and TreeProcessingRouter.
   * Currently TableProcessingRouter can handle only GlobalIndex processing.
   */
  class EventRouter {
  public:
    virtual ~EventRouter() {};
    virtual void buildTable () {};
    /* remedius
     * insertEvent method was renamed to processEvent method,
     * since we've introduced event processing on the input side as well.
     */
    virtual void insertRoutingData (Interval& i, OutputRoutingData& data) {}
    virtual void insertRoutingData (Interval& i, InputRoutingData<EventHandlerGlobalIndex>& data) {}
    virtual void insertRoutingData (Interval& i, InputRoutingData<EventHandlerLocalIndex>& data) {}
    virtual void processEvent (double t, int id) {};
    virtual bool needFewPoints () const { return false; }
    virtual DirectRouter* directRouter () { return 0; }; //Remove this later
  };

  template<class RoutingData, class IntervalLookup>
  class IntervalProcessingRouter : public EventRouter {
    class Processor : public IntervalLookup::Action {
    protected:
      double t_;
      int id_;
    public:
      Processor (double t, int id) : t_ (t), id_ (id) { };
      void operator() (RoutingData& data)
      {
	/* remedius
	 * for Global index ((RoutingData &)data).offset () is set to 0 during
	 * SpatialNegotiator::wrapIntervals();
	 */
	data.process (t_, id_ - data.offset ());
      }
    };
    
    IntervalLookup routingTable;
  public:
    void insertRoutingData (Interval& i, RoutingData& data);
    void buildTable ();
    void processEvent (double t, int id);
  };

  template<class RoutingData, class IntervalLookup>
  void
  IntervalProcessingRouter<RoutingData, IntervalLookup>::insertRoutingData (Interval& i,
									    RoutingData &data)
  {
    routingTable.add (i, data);
  }


  template<class RoutingData, class IntervalLookup>
  void
  IntervalProcessingRouter<RoutingData, IntervalLookup>::buildTable ()
  {
    routingTable.build ();
  }

  template<class RoutingData, class IntervalLookup>
  void
  IntervalProcessingRouter<RoutingData, IntervalLookup>::processEvent (double t, int id)
  {
    Processor i (t, id);
    routingTable.search (id, &i);
  }

  class TreeProcessingOutputRouter
    : public IntervalProcessingRouter<OutputRoutingData,
				      IntervalTree<int, Interval, OutputRoutingData> >
  {
  };

  class TreeProcessingInputGlobalRouter
    : public IntervalProcessingRouter<InputRoutingData<EventHandlerGlobalIndex>,
				      IntervalTree<int,
						   Interval,
						   InputRoutingData<EventHandlerGlobalIndex> > >
  {
  };

  class TreeProcessingInputLocalRouter
    : public IntervalProcessingRouter<InputRoutingData<EventHandlerLocalIndex>,
				      IntervalTree<int,
						   Interval,
						   InputRoutingData<EventHandlerLocalIndex> > >
  {
  };

  class TableProcessingOutputRouter
    : public IntervalProcessingRouter<OutputRoutingData,
				      IntervalTable<int, Interval, OutputRoutingData> >
  {
  public:
    bool needFewPoints () const { return true; }
  };

  class TableProcessingInputGlobalRouter
    : public IntervalProcessingRouter<InputRoutingData<EventHandlerGlobalIndex>,
				      IntervalTable<int,
						    Interval,
						    InputRoutingData<EventHandlerGlobalIndex> > >
  {
  public:
    bool needFewPoints () const { return true; }
  };

  class TableProcessingInputLocalRouter
    : public IntervalProcessingRouter<InputRoutingData<EventHandlerLocalIndex>,
				      IntervalTable<int,
						    Interval,
						    InputRoutingData<EventHandlerLocalIndex> > >
  {
  public:
    bool needFewPoints () const { return true; }
  };

  class DirectRouter : public EventRouter {
    char* buffer_;
    unsigned int size_;
    unsigned int pos_;
    std::vector<char> extra_;
  public:
    DirectRouter () : size_ (0), pos_ (0) { }
    DirectRouter* directRouter () { return this; }; //Remove this later
    unsigned int dataSize () { return pos_ + extra_.size (); }
    inline void processEvent (double t, int id)
    {
      if (pos_ < size_)
	{
	  Event* e = static_cast<Event*> (static_cast<void*> (buffer_ + pos_));
	  e->t = t;
	  e->id = id;
	  pos_ += sizeof (Event);
	}
      else
	processExtra (t, id);
    }
    void processExtra (double t, int id);
    void setOutputBuffer (void* buffer, unsigned int size);
    void fillOutputBuffer () { pos_ = 0; }
  };

  class HybridTreeProcessingOutputRouter
    : public TreeProcessingOutputRouter
  {
  private:
    DirectRouter directRouter_;
  public:
    DirectRouter* directRouter () { return &directRouter_; }
    inline void processEvent (double t, int id)
    {
      TreeProcessingOutputRouter::processEvent (t, id);
      directRouter_.processEvent (t, id);
    }
  };

  class HybridTableProcessingOutputRouter
    : public TableProcessingOutputRouter
  {
  private:
    DirectRouter directRouter_;
  public:
    DirectRouter* directRouter () { return &directRouter_; }
    inline void processEvent (double t, int id)
    {
      TableProcessingOutputRouter::processEvent (t, id);
      directRouter_.processEvent (t, id);
    }
  };

}
#define MUSIC_EVENT_ROUTER_HH
#endif
