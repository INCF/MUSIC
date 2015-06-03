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
#include <cstring>
#include <iostream>
#include "music/FIBO.hh"

namespace MUSIC {

  FIBO::FIBO (int es)
  {
    configure (es);
  }

  
  void
  FIBO::configure (int es)
  {
    elementSize = es;
    size_ = elementSize * nInitial;
    if (es > 0)
      buffer.resize (size_);
    current = 0;
  }

  
  bool
  FIBO::isEmpty ()
  {
    return current == 0;
  }

  
  void*
  FIBO::insert ()
  {
    if (current == size_)
      grow (2 * size_);
    // Here we use the assumption that vector memory is contiguous
    // Josuttis says this is the intention of STL even though the
    // first version of the report is not clear about this.
    void* memory = static_cast<void*> (&buffer[current]);
    current += elementSize;
    return memory;
  }
  

  void
  FIBO::insert (void* elements, int n_elements)
  {
    int blockSize = elementSize * n_elements;
    if (current + blockSize > size_)
      grow (3 * (current + blockSize) / 2);
    // Here we use the assumption that vector memory is contiguous
    // Josuttis says this is the intention of STL even though the
    // first version of the report is not clear about this.
    void* memory = static_cast<void*> (&buffer[current]);
    memcpy (memory, elements, blockSize);
    current += blockSize;
  }


  void
  FIBO::clear ()
  {
    current = 0;
  }
  

  void
  FIBO::nextBlockNoClear (void*& data, int& blockSize)
  {

    data = static_cast<void*> (&buffer[0]);
    blockSize = current;
  }


  void
  FIBO::nextBlock (void*& data, int& blockSize)
  {
    nextBlockNoClear (data, blockSize);
    clear ();
  }


  void
  FIBO::grow (int newSize)
  {
    size_ = newSize;
    buffer.resize (size_);
  }
  
}
