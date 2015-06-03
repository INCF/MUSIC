/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2008, 2009 INCF
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

#ifndef MUSIC_INTERVAL_HH

#include <ostream>

namespace MUSIC {

  class Interval {
    int begin_;
    int end_;
  public:
    Interval () { }
    Interval (int b, int e) : begin_ (b), end_ (e) { }
    ~Interval () { }
    int begin () const { return begin_; }
    int end () const { return end_; }
    void setBegin (int begin) { begin_ = begin; }
    void setEnd (int end) { end_ = end; }
    bool operator< (const Interval& data) const
    {
      return begin () < data.begin ();
    }

  };

  // support printed representation
  std::ostream& operator<< (std::ostream& os, const Interval& ival);

}
#define MUSIC_INTERVAL_HH
#endif
