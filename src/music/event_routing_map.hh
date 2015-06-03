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
/* remedius
 * event_routingmap.hh and event_routingmap.cc were added to the repository
 * as a new hierarchy for EventRoutingMap class.
 * Before EventRoutingMap class was defined in event_router.cc file.
 * EventRoutingMap class keeps the former methods however it became a template
 * due to different data types needed for processing events on the receiver and sender sides.
 */
#ifndef MUSIC_EVENT_ROUTINGMAP_HH
#include <map>
#include <vector>
#include <music/FIBO.hh>
#include <music/interval_tree.hh>
#include <music/index_map.hh>
#include <music/event.hh>

namespace MUSIC {

  class EventRouter;

  template <class DataType>
  class EventRoutingMap{
  public:
    virtual ~EventRoutingMap ();
    virtual void fillRouter (EventRouter *router)=0;
    virtual void insert (IndexInterval i, DataType data);
  protected:
    typedef std::map<DataType, std::vector<IndexInterval *> >DataMap;
    DataMap dataMap;
    EventRoutingMap () {}
    std::vector<IndexInterval> rebuildIntervals (std::vector<IndexInterval*> &intervals);
    struct comp{
      bool operator() (IndexInterval* i,IndexInterval *j) { return (*i<*j);}
    } comp_obj;


  };
  template<class DataType>
  EventRoutingMap<DataType>::~EventRoutingMap(){
    typename DataMap::iterator pos;
    for (pos = dataMap.begin (); pos != dataMap.end (); ++pos)
      for (std::vector<IndexInterval *>::iterator i =
	     pos->second.begin (); i != pos->second.end (); i++)
	delete (*i);
  }
  template<class DataType>
  void
  EventRoutingMap<DataType>::insert (IndexInterval i, DataType data)
  {
    dataMap[data].push_back (new IndexInterval(i.begin(),i.end(),i.local()));
  }
  template<class DataType>
  std::vector<IndexInterval>
  EventRoutingMap<DataType>::rebuildIntervals (std::vector<IndexInterval*> &intervals)
  {
    std::vector<IndexInterval> newIntervals;

    // Sort all intervals
    sort (intervals.begin (), intervals.end (), comp_obj);
    std::vector<IndexInterval *>::iterator i = intervals.begin ();


    // Build sequence of unions out of the original interval sequence
    i = intervals.begin ();
    while (i != intervals.end ())
      {
	IndexInterval current = **i++;

	while (i != intervals.end ()
	       && (*i)->begin () <= current.end ())
	  {
	    // join intervals
	    int maxEnd = std::max<int> (current.end (), (*i)->end ());
	    current.setEnd (maxEnd);
	    ++i;
	  }

	newIntervals.push_back (current);
      }

    return newIntervals;
  }
  /* remedius
   * InputRoutingMap is an EventRoutingMap that's used for processing the events on the receiver side
   */

  class InputRoutingMap:public EventRoutingMap<EventHandlerPtr*>
  {
  public:
    void fillRouter (EventRouter *router);
  private:
    void insertRoutingInterval(EventRouter *router, IndexInterval i, EventHandlerPtr *h);
  };

  /* remedius
   * OutputRoutingMap is an EventRoutingMap that's used for processing the events on the sender side
   */
  class OutputRoutingMap:public EventRoutingMap<FIBO*>
  {
    std::vector<IndexInterval*> *intervals;
  public:
    OutputRoutingMap(){intervals = new std::vector<IndexInterval *>;}
    ~OutputRoutingMap();
    void fillRouter (EventRouter *router);
    void insert (IndexInterval i, FIBO* data);
  private:
    void insertRoutingInterval(EventRouter *router, IndexInterval i, FIBO *b);
  };
}
#define MUSIC_EVENT_ROUTINGMAP_HH
#endif
