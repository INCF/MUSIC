/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2009, 2022 INCF
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

#include "music/sampler.hh"

#include "music/debug.hh"

#if MUSIC_USE_MPI

// array_data.hh needs to be included first since it causes inclusion
// of mpi.h (in data_map.hh).  mpi.h must be included before other
// header files on BG/L
#include "music/array_data.hh"
#include "music/index_map_factory.hh"
#include "music/error.hh"
#include "music/mpi_utils.hh"

#include <cstring>



namespace MUSIC {

  Sampler::Sampler ()
    : dataMap_ (0), interpolationDataMap_ (0)
  {
  }


  Sampler::~Sampler ()
  {
    if (dataMap_ != 0)
      delete dataMap_;
    if (interpolationDataMap_ != 0)
      delete interpolationDataMap_;
  }
  

  /*
   * Called during port mapping
   */
  void
  Sampler::configure (DataMap* dataMap)
  {
    dataMap_ = dataMap->copy ();
  }


  void
  Sampler::initialize ()
  {
    elementSize = mpi_get_type_size (dataMap_->type ());
    
    size = 0;
    IndexMap* indices = dataMap_->indexMap ();
    IndexMapFactory newIndices;
    for (IndexMap::iterator i = indices->begin ();
	 i != indices->end ();
	 ++i)
      {
	int localIndex = size;
	newIndices.add (i->begin (), i->end (), localIndex);
	size += i->end () - i->begin ();
      }
    newIndices.build ();

    prevSample_ = new ContDataT[elementSize * size];
    sample_ = new ContDataT[elementSize * size];
    interpolationData_ = new ContDataT[elementSize * size];

    interpolationDataMap_ = new ArrayData (interpolationData_,
					   dataMap_->type (),
					   &newIndices);

    MUSIC_LOGR ("prev = " << static_cast<void*> (prevSample_)
		<< ", sample = " << static_cast<void*> (sample_)
		<< ", interp = " << static_cast<void*> (interpolationData_));
  }

  
  /*
   * Called during configuration of interpolating connectors
   */
  DataMap*
  Sampler::interpolationDataMap ()
  {
    if (interpolationDataMap_ == 0)
      initialize ();
    
    return interpolationDataMap_;
  }


  void
  Sampler::newSample ()
  {
    hasSampled = false;
  }


  void
  Sampler::sampleOnce ()
  {
    if (!hasSampled)
      {
	sample ();
	hasSampled = true;
      }
  }


  void
  Sampler::sample ()
  {
    ContDataT* dest = insert ();

    int pos = 0;
    IndexMap* indices = dataMap_->indexMap ();
    for (IndexMap::iterator i = indices->begin ();
	 i != indices->end ();
	 ++i)
      {
	ContDataT* src = static_cast<ContDataT*> (dataMap_->base ());
	int iSize = elementSize * (i->end () - i->begin ());
	memcpy (dest + pos,
		src + elementSize * (i->begin () - i->local ()),
		iSize);
	pos += iSize;
      }
  }

  
  ContDataT*
  Sampler::insert ()
  {
    swapBuffers (sample_, prevSample_);
    return sample_;
  }

  
  void
  Sampler::swapBuffers (ContDataT*& b1, ContDataT*& b2)
  {
    ContDataT* tmp;
    tmp = b1;
    b1 = b2;
    b2 = tmp;
  }


  void
  Sampler::interpolate (double interpolationCoefficient)
  {
    interpolateTo (interpolationDataMap_, interpolationCoefficient);
  }
  
  
  void
  Sampler::interpolateToApplication (double interpolationCoefficient)
  {
    interpolateTo (dataMap_, interpolationCoefficient);
  }
  

  void
  Sampler::interpolateTo (DataMap* dataMap, double interpolationCoefficient)
  {
    int pos = 0;
    IndexMap* indices = dataMap->indexMap ();
    for (IndexMap::iterator i = indices->begin ();
	 i != indices->end ();
	 ++i)
      {
	int localIndex = i->begin () - i->local ();
	int iSize = i->end () - i->begin ();
	if (dataMap->type () == MPI_DOUBLE)
	  interpolate (pos, iSize, interpolationCoefficient,
		       static_cast<double*> (dataMap->base ()) + localIndex);
	else if (dataMap->type () == MPI_FLOAT)
	  interpolate (pos, iSize, interpolationCoefficient,
		       static_cast<float*> (dataMap->base ()) + localIndex);
	else
	  error ("internal error in Sampler::interpolateTo");
	pos += iSize;
      }    
  }


  void
  Sampler::interpolate (int from,
			int n,
			double interpolationCoefficient,
			double* dest)
  {
    MUSIC_LOGR ("interpolate to dest = " << static_cast<void*> (dest)
		<< ", begin = " << from
		<< ", length = " << n);
    double* prev = (static_cast<double*> (static_cast<void*> (prevSample_))
		    + from);
    double* succ = (static_cast<double*> (static_cast<void*> (sample_))
		    + from);
    for (int i = 0; i < n; ++i)
      dest[i] = ((1.0 - interpolationCoefficient) * prev[i]
		 + interpolationCoefficient * succ[i]);
  }
			
  
  void
  Sampler::interpolate (int from,
			int n,
			float interpolationCoefficient,
			float* dest)
  {
    float* prev = (static_cast<float*> (static_cast<void*> (prevSample_))
		    + from);
    float* succ = (static_cast<float*> (static_cast<void*> (sample_))
		    + from);
    for (int i = 0; i < n; ++i)
      dest[i] = ((1.0 - interpolationCoefficient) * prev[i]
		 + interpolationCoefficient * succ[i]);
  }
			
  
}
#endif
