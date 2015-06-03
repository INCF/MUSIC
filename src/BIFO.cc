/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2009 INCF
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

#include "music/BIFO.hh"

#include "music/debug.hh"

#if MUSIC_USE_MPI

#include <cstring>
#include "music/error.hh"

namespace MUSIC {

  void
  BIFO::configure (int elementSize, int maxBlockSize)
  {
    elementSize_ = elementSize;
    maxBlockSize_ = maxBlockSize;
    size = maxBlockSize_;
    buffer.resize (size);
    beginning = 0;
    end = 0;
    current = 0;
    top = 0;

  }


  // Duplicate the single element in the buffer to a total of nElements
  // 0 is allowed as argument in which case the buffer is emptied
  void
  BIFO::fill (int nElements)
  {
    int neededSize = nElements * elementSize_;
    if (neededSize > size)
      grow (neededSize);

    if (end - current != elementSize_)
      error ("internal error: BIFO in erroneous state before fill");
    
    beginning = 0;
    
    if (nElements == 0)
      {
	end = 0;
	current = 0;
	top = 0;
	return;
      }

    // Here we use the assumption that vector memory is contiguous
    // Josuttis says this is the intention of STL even though the
    // first version of the report is not clear about this.

    if (current != 0)
      {
	memcpy (&buffer[0], &buffer[current], elementSize_);
	current = 0;
      }
    
    top = elementSize_;
    end = nElements * elementSize_;
    while (top < end)
      {
	memcpy (&buffer[top], &buffer[0], elementSize_);
	top += elementSize_;
      }
  }

  
  bool
  BIFO::isEmpty ()
  {
    return current == end;
  }


  void*
  BIFO::insertBlock ()
  {
    beginning = end; // set insertion point to end of last block
    if (current <= end) // reading below inserting?
      {
	// Inserting above current data


	// Is it time to wrap around the insertion point?  This should
	// only be done if we can fit a maximal block below current
	// (current > maxBlockSize_).  We need to use > since a
	// maximal block otherwise could cause an empty buffer.
	if (current > maxBlockSize_)
	  beginning = 0; // Wrap around!
	else if (beginning + maxBlockSize_ > size) // Need to enlarge?
	  // (The peculiar choice of growing just as much as we need
	  // is made since buffers can be VERY large in large scale
	  // simulations.)
	  grow (beginning + maxBlockSize_);
      }
    else
      {
	// Inserting below current data
	if (current - beginning <= maxBlockSize_) // Too tight?
	  {
	    // delta is how much more space we need + epsilon (8 to be
	    // nice to memory)
	    int delta = maxBlockSize_ - (current - beginning) + 8;
	    if (size - top < delta) // Need to enlarge?
	    // (The peculiar choice of growing just as much as we need
	    // is made since buffers can be VERY large in large scale
	    // simulations.)
	      grow (size + delta);
	    // Now move entire upper chunk of data:
	    memmove (&buffer[current + delta], &buffer[current],
		     top - current);
	    current += delta;
	    top += delta;
	  }
      }
    return static_cast<void*> (&buffer[beginning]);
  }


  void
  BIFO::trimBlock (int blockSize)
  {
    if (blockSize > 0)
      {
	end = beginning + blockSize;
	if (end > size)
	  {
	    std::ostringstream msg;
	    msg << "BIFO buffer overflow; received "
		<< blockSize / elementSize_
		<< ", room " << (size - beginning) / elementSize_
		<< ", permitted " << maxBlockSize_ / elementSize_;
	    error (msg);
	  }
	if (current <= end)
	  top = end;
      }
  }


  void*
  BIFO::next ()
  {
    if (isEmpty ())
      {
	MUSIC_LOGR ("attempt to read from empty BIFO buffer");
	return NULL;
      }
    
    if (current == top)
      {
	// wrap around
	current = 0;
	top = end;
      }
    
    // Here we use the assumption that vector memory is contiguous
    // Josuttis says this is the intention of STL even though the
    // first version of the report is not clear about this.
    void* memory = static_cast<void*> (&buffer[current]);
    /* remedius
     * could it be possible that elementSize > size of received data? no
     * changed from current+=elementSize_ to the following line:
     */
    current += std::min(elementSize_,top-current);
    return memory;
  }

  void
  BIFO::grow (int newSize)
  {
    size = newSize;
    buffer.resize (size);
  }
    
}
#endif
