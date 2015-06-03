/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2012 INCF
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

#include <iostream>

extern "C" {
#include <malloc.h>
}

#include "music/memory.hh"

#include "../config.h"

namespace MUSIC {

  void
  reportMem ()
  {
#ifdef HAVE_MALLINFO
    struct mallinfo minfo = mallinfo ();
    std::cout << "Allocated with sbrk (arena): " << minfo.arena << std::endl;
    std::cout << "Allocated with mmap (hblkhd): " << minfo.hblkhd << std::endl;
    std::cout << "Chunks handed out by malloc (uordblks): " << minfo.uordblks << std::endl;
    std::cout << "Free memory in pool (fordblks): " << minfo.fordblks << std::endl;
#endif
  }

}
