/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2009 CSC, KTH
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

namespace MUSIC {

  class spike {
  public:
    double t;
    int id;
  };

  class spike_fifo : public fifo<spike> {
  public:
    void insert (int id, double t)
    {
      spike& s = fifo<spike>::insert ();
      s.id = id;
      s.t = t;
    }
  };    

}
