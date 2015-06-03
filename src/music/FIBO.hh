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

#ifndef MUSIC_FIBO_HH
#include "music/music-config.hh"
#include <vector>

namespace MUSIC {

  class FIBO {
  private:
    static const int nInitial = 10;
    
    std::vector<char> buffer;
    int elementSize;
    int size_;
    int current;

    void grow (int newSize);
    
  public:
    FIBO () { }
    FIBO (int elementSize);
    void configure (int elementSize);
    bool isEmpty ();
    // NOTE: find better return type
    void* insert ();
    void insert (void* elements, int n_elements);
    void clear ();
    void nextBlockNoClear (void*& data, int& size);
    void nextBlock (void*& data, int& size);
    unsigned int size () { return size_; } //*fixme* should be n elements
  };
  
  
}
#define MUSIC_FIBO_HH
#endif
