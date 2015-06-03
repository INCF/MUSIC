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
#include "music/clock.hh"

namespace MUSIC {

  ClockState::ClockState (double t, double tb)
  {
    double dstate = t / tb;
    if (dstate >= 0.0)
      state = dstate + 0.5;
    else
      state = dstate - 0.5;
  }

  
  ClockState
  ClockState::Serialized::deserialize ()
  {
    return (static_cast<long long> (upper) << 32) | lower;
  }


  ClockState::Serialized
  ClockState::serialize ()
  {
    Serialized s;
    s.upper = (state >> 32);
    s.lower = state & 0xffffffffLL;
    return s;
  }


  Clock::Clock (double tb, double h)
  {
    configure (tb, ClockState (h, tb));
  }


  void
  Clock::configure (double tb, ClockState ti)
  {
    state_ = 0;
    timebase_ = tb;
    tickInterval_ = ti;
  }


  void
  Clock::reset ()
  {
    state_ = 0;
  }


  void
  Clock::tick ()
  {
    state_ += tickInterval_;
  }
  

  void
  Clock::ticks (int n)
  {
    state_ += n * tickInterval_;
  }
  

  double
  Clock::time () const
  {
    return timebase_ * state_;
  }
  
}
