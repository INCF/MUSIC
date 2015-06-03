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

#ifndef MUSIC_CLOCK_HH
#include "music/music-config.hh"

namespace MUSIC {

  // ClockState may be negative due to calculations in Synchronizer
  class ClockState {
#ifdef MUSIC_HAVE_LONG_LONG
    long long state;
#else
#error 64-bit clocks without long long not yet implemented
#endif
  public:
    ClockState () { }
    // The following operations should be defined in this header file
    // so that they can be inlined by the compiler.
    inline ClockState (const long long s) : state (s) { }
    ClockState (double t, double tb);
    inline operator long long () const { return state; }
    inline ClockState& operator+= (const ClockState& s)
    {
      state += s;
      return *this;
    }
    class Serialized;
    Serialized serialize ();
    // The serialized representation of a ClockState is guaranteed to
    // fit in two sequential long values
    class Serialized {
      friend Serialized ClockState::serialize ();

    public:
      signed long upper;
      unsigned long lower;
    public:
      ClockState deserialize ();
    };
  };
  
  class Clock {
    ClockState state_;
    ClockState tickInterval_;
    double timebase_;
  public:
    Clock () { state_ = 0; };
    Clock (double tb, double h);
    void configure (double tb, ClockState ti);
    void reset ();
    void tick ();
    void ticks (int n);
    ClockState tickInterval () { return tickInterval_; }
    void setTickInterval (ClockState ti) { tickInterval_ = ti; }
    double timebase () { return timebase_; }
    double time () const;
    ClockState integerTime () { return state_; }
    void set (ClockState state) { state_ = state; }
    bool operator>= (const Clock& ref) const { return state_ >= ref.state_; }
    bool operator<= (const Clock& ref) const { return state_ <= ref.state_; }
    bool operator< (const Clock& ref) const { return state_ < ref.state_; }
    bool operator!= (const Clock& ref) const {return state_ != ref.state_;}
    bool operator== (const Clock& ref) const { return state_ == ref.state_; }
  };

}
#define MUSIC_CLOCK_HH
#endif
