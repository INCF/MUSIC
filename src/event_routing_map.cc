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

//#define MUSIC_DEBUG
//#include "music/music-config.hh"

#include "music/event_router.hh"
#include "music/event_routing_map.hh"

namespace MUSIC {

  void
  OutputRoutingMap::insertRoutingInterval (EventRouter *router, IndexInterval i, FIBO *b)
  {
    OutputRoutingData data_ (i, b);
    router->insertRoutingData (i, data_);
  }

  OutputRoutingMap::~OutputRoutingMap()
  {
    delete intervals;
  }

  void
  OutputRoutingMap::insert (IndexInterval i, FIBO* data)
  {
    EventRoutingMap<FIBO*>::insert (i, data);
    intervals->push_back (dataMap[data].back ());
  }

  void
  OutputRoutingMap::fillRouter (EventRouter *router)
  {
    std::vector<IndexInterval> new_intervals = rebuildIntervals (*intervals);
    //std::vector<IndexInterval*> new_intervals = *intervals;
    std::map<FIBO*, std::vector<IndexInterval *> >::iterator pos;
    for (pos = dataMap.begin (); pos != dataMap.end (); ++pos)
      {
	sort (pos->second.begin (), pos->second.end (), comp_obj);

	std::vector<IndexInterval*>::iterator i = pos->second.begin ();

	if (router->needFewPoints ())
	  {
	    // insert intervals without trying to close gaps
	    while (i != pos->second.end ())
	      insertRoutingInterval (router, **i++, pos->first);
	    continue;
	  }
	
	std::vector<IndexInterval>::iterator mapped = new_intervals.begin ();
	while (i != pos->second.end ())
	  {
	    IndexInterval current = **i++;
	    while (i != pos->second.end ()
		   && (*i)->local () == current.local ())
	      {
		// Define the gap between current and next interval
		int gapBegin = current.end ();
		int gapEnd = (*i)->begin ();

		// Skip mapped intervals which end before gap
		while (mapped != new_intervals.end ()
		       && (*mapped).end () <= gapBegin){

		  ++mapped;
		}
		// Check that gap does not overlap with any mapped interval
		if (mapped != new_intervals.end () && (*mapped).begin () < gapEnd)
		  break;
		// Join intervals by closing over gap
		current.setEnd ((*i)->end ());
		++i;
	      }
	    insertRoutingInterval (router, current, pos->first);
	  }
      }
  }


  void
  InputRoutingMap::insertRoutingInterval (EventRouter *router,
					  IndexInterval i,
					  EventHandlerPtr *h)
  {
    if (h->getType () == Index::GLOBAL)
      {
	InputRoutingData<EventHandlerGlobalIndex> data_ (i, h);
	router->insertRoutingData (i, data_);
      }
    else
      {
	InputRoutingData<EventHandlerLocalIndex> data_ (i, h);
	router->insertRoutingData (i, data_);
      }
  }

  void InputRoutingMap::fillRouter (EventRouter *router)
  {
    std::map<EventHandlerPtr*, std::vector<IndexInterval *> >::iterator pos;
    for (pos = dataMap.begin (); pos != dataMap.end (); ++pos)
      {
	std::vector<IndexInterval > new_intervals = rebuildIntervals (((*pos).second));
	std::vector<IndexInterval >::iterator i = new_intervals.begin ();
	while (i != new_intervals.end ())
	  insertRoutingInterval (router, *i++, (*pos).first);
      }
  }
}
