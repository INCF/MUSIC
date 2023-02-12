/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009, 2022 INCF
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

#include "music/event.hh"
#include "music/event_router.hh"

#include <algorithm>
#include <cassert>
#include <cstring>

namespace MUSIC {

  OutputRoutingData::OutputRoutingData (const IndexInterval &i, FIBO* b) : EventRoutingData(i), buffer_ (b)
  {
  }

  void
  OutputRoutingData::process (double t, int id)
  {
    Event* e = static_cast<Event*> (buffer_->insert ());
    e->t = t;
    e->id = id;
  }

  void
  DirectRouter::processExtra (double t, int id)
  {
    unsigned int pos = extra_.size ();
    extra_.resize (pos + sizeof (Event));
    Event* e = static_cast<Event*> (static_cast<void*> (&extra_[pos]));
    e->t = t;
    e->id = id;
  }

  void
  DirectRouter::setOutputBuffer (void* buffer, unsigned int size)
  {
    if (size == size_)
      {
	// Buffer relocated
	buffer_ = static_cast<char*> (buffer);
	return;
      }
#if 0
    if (size - size_ != extra_.size ())
      std::cout << "Rank " << mpi_get_rank (MPI_COMM_WORLD)
		<< ": DirectRouter: Had " << extra_.size ()
		<< " extra spike space, size changed from "
		<< size_ << " to " << size << " = " << size - size_
		<< std::endl;
#endif
    assert (size - size_ == extra_.size ());
    buffer_ = static_cast<char*> (buffer);
    memcpy (buffer_ + size_, &extra_[0], extra_.size ());
    extra_.clear ();
    std::vector<char> (extra_).swap (extra_); // trim
    size_ = size;
  }

}
