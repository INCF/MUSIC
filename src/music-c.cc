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
#include "music.hh"
#if MUSIC_USE_MPI


#include <string>
#include <cstring>

extern "C" {

#include "music-c.h"

/* Setup */

MUSIC_Setup *
MUSIC_createSetup (int *argc, char ***argv)
{
  return (MUSIC_Setup *) new MUSIC::Setup (*argc, *argv);
}


MUSIC_Setup *
MUSIC_createSetupThread (int *argc, char ***argv, int required, int *provided)
{
  return (MUSIC_Setup *) new MUSIC::Setup (*argc, *argv, required, provided);
}


/* Communicators */

MPI_Comm
MUSIC_setupCommunicatorGlue (MUSIC_Setup *setup)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return (MPI_Comm) cxxSetup->communicator ();
}


/* Port creation */

MUSIC_ContOutputPort *
MUSIC_publishContOutput (MUSIC_Setup *setup, char *id)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return (MUSIC_ContOutputPort *) cxxSetup->publishContOutput (id);
}


MUSIC_ContInputPort *
MUSIC_publishContInput (MUSIC_Setup *setup, char *id)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return (MUSIC_ContInputPort *) cxxSetup->publishContInput(id);
}


MUSIC_EventOutputPort *
MUSIC_publishEventOutput (MUSIC_Setup *setup, char *id)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return (MUSIC_EventOutputPort *) cxxSetup->publishEventOutput(id);
}


MUSIC_EventInputPort *
MUSIC_publishEventInput (MUSIC_Setup *setup, char *id)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return (MUSIC_EventInputPort *) cxxSetup->publishEventInput(id);
}


MUSIC_MessageOutputPort *
MUSIC_publishMessageOutput (MUSIC_Setup *setup, char *id)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return (MUSIC_MessageOutputPort *) cxxSetup->publishMessageOutput(id);
}


MUSIC_MessageInputPort *
MUSIC_publishMessageInput (MUSIC_Setup *setup, char *id)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return (MUSIC_MessageInputPort *) cxxSetup->publishMessageInput(id);
}

/*
void
MUSIC_destroyContOutput (MUSIC_ContOutputPort* Port)
{
  delete (MUSIC::ContOutputPort *) Port;
}


void
MUSIC_destroyContInput (MUSIC_ContInputPort* Port)
{
  delete (MUSIC::ContInputPort *) Port;
}


void
MUSIC_destroyEventOutput (MUSIC_EventOutputPort* Port)
{
  delete (MUSIC::EventOutputPort *) Port;
}


void
MUSIC_destroyEventInput (MUSIC_EventInputPort* Port)
{
  delete (MUSIC::EventInputPort *) Port;
}


void
MUSIC_destroyMessageOutput (MUSIC_MessageOutputPort* Port)
{
  delete (MUSIC::MessageOutputPort *) Port;
}


void
MUSIC_destroyMessageInput (MUSIC_MessageInputPort* Port)
{
  delete (MUSIC::MessageInputPort *) Port;
}
*/

/* General port methods */

int
MUSIC_ContOutputPort_isConnected (MUSIC_ContOutputPort *Port)
{
  MUSIC::ContOutputPort* cxxPort = (MUSIC::ContOutputPort *) Port;
  return cxxPort->isConnected ();
}


int
MUSIC_ContInputPort_isConnected (MUSIC_ContInputPort *Port)
{
  MUSIC::ContInputPort* cxxPort = (MUSIC::ContInputPort *) Port;
  return cxxPort->isConnected ();
}


int
MUSIC_EventOutputPort_isConnected (MUSIC_EventOutputPort *Port)
{
  MUSIC::EventOutputPort* cxxPort = (MUSIC::EventOutputPort *) Port;
  return cxxPort->isConnected ();
}


int
MUSIC_EventInputPort_isConnected (MUSIC_EventInputPort *Port)
{
  MUSIC::EventInputPort* cxxPort = (MUSIC::EventInputPort *) Port;
  return cxxPort->isConnected ();
}


int
MUSIC_MessageOutputPort_isConnected (MUSIC_MessageOutputPort *Port)
{
  MUSIC::MessageOutputPort* cxxPort = (MUSIC::MessageOutputPort *) Port;
  return cxxPort->isConnected ();
}


int
MUSIC_MessageInputPort_isConnected (MUSIC_MessageInputPort *Port)
{
  MUSIC::MessageInputPort* cxxPort = (MUSIC::MessageInputPort *) Port;
  return cxxPort->isConnected ();
}


int
MUSIC_ContOutputPort_hasWidth (MUSIC_ContOutputPort *Port)
{
  MUSIC::ContOutputPort* cxxPort = (MUSIC::ContOutputPort *) Port;
  return cxxPort->hasWidth ();
}


int
MUSIC_ContInputPort_hasWidth (MUSIC_ContInputPort *Port)
{
  MUSIC::ContInputPort* cxxPort = (MUSIC::ContInputPort *) Port;
  return cxxPort->hasWidth ();
}


int
MUSIC_EventOutputPort_hasWidth (MUSIC_EventOutputPort *Port)
{
  MUSIC::EventOutputPort* cxxPort = (MUSIC::EventOutputPort *) Port;
  return cxxPort->hasWidth ();
}


int
MUSIC_EventInputPort_hasWidth (MUSIC_EventInputPort *Port)
{
  MUSIC::EventInputPort* cxxPort = (MUSIC::EventInputPort *) Port;
  return cxxPort->hasWidth ();
}


int
MUSIC_ContOutputPort_width (MUSIC_ContOutputPort *Port)
{
  MUSIC::ContOutputPort* cxxPort = (MUSIC::ContOutputPort *) Port;
  return cxxPort->width ();
}


int
MUSIC_ContInputPort_width (MUSIC_ContInputPort *Port)
{
  MUSIC::ContInputPort* cxxPort = (MUSIC::ContInputPort *) Port;
  return cxxPort->width ();
}


int
MUSIC_EventOutputPort_width (MUSIC_EventOutputPort *Port)
{
  MUSIC::EventOutputPort* cxxPort = (MUSIC::EventOutputPort *) Port;
  return cxxPort->width ();
}


int
MUSIC_EventInputPort_width (MUSIC_EventInputPort *Port)
{
  MUSIC::EventInputPort* cxxPort = (MUSIC::EventInputPort *) Port;
  return cxxPort->width ();
}


/* Mapping */

/* No arguments are optional. */

void
MUSIC_ContOutputPort_map (MUSIC_ContOutputPort *Port,
			  MUSIC_ContData *dmap,
			  int maxBuffered)
{
  MUSIC::ContOutputPort* cxxPort = (MUSIC::ContOutputPort *) Port;
  MUSIC::ContData* cxxDmap = (MUSIC::ContData *) dmap;
  cxxPort->map (cxxDmap, maxBuffered);
}


void
MUSIC_ContInputPort_map (MUSIC_ContInputPort *Port,
			 MUSIC_ContData *dmap,
			 double delay,
			 int maxBuffered,
			 int interpolate)
{
  MUSIC::ContInputPort* cxxPort = (MUSIC::ContInputPort *) Port;
  MUSIC::ContData* cxxDmap = (MUSIC::ContData *) dmap;
  cxxPort->map (cxxDmap, delay, maxBuffered, interpolate);
}


void
MUSIC_EventOutputPort_mapGlobalIndex (MUSIC_EventOutputPort *Port,
				      MUSIC_IndexMap *indices,
				      int maxBuffered)
{
  MUSIC::EventOutputPort* cxxPort = (MUSIC::EventOutputPort *) Port;
  MUSIC::IndexMap* cxxIndices = (MUSIC::IndexMap *) indices;
  cxxPort->map (cxxIndices, MUSIC::Index::GLOBAL, maxBuffered);
}


void
MUSIC_EventOutputPort_mapLocalIndex (MUSIC_EventOutputPort *Port,
				     MUSIC_IndexMap *indices,
				     int maxBuffered)
{
  MUSIC::EventOutputPort* cxxPort = (MUSIC::EventOutputPort *) Port;
  MUSIC::IndexMap* cxxIndices = (MUSIC::IndexMap *) indices;
  cxxPort->map (cxxIndices, MUSIC::Index::LOCAL, maxBuffered);
}


typedef void MUSIC_EventHandler (double t, int id);

void
MUSIC_EventInputPort_mapGlobalIndex (MUSIC_EventInputPort *Port,
				     MUSIC_IndexMap *indices,
				     MUSIC_EventHandler *handleEvent,
				     double accLatency,
				     int maxBuffered)
{
  MUSIC::EventInputPort* cxxPort = (MUSIC::EventInputPort *) Port;
  MUSIC::IndexMap* cxxIndices = (MUSIC::IndexMap *) indices;
  MUSIC::EventHandlerGlobalIndexProxy* cxxHandleEvent =
    cxxPort->allocEventHandlerGlobalIndexProxy (handleEvent);
  cxxPort->map (cxxIndices, cxxHandleEvent, accLatency, maxBuffered);
}


void
MUSIC_EventInputPort_mapLocalIndex (MUSIC_EventInputPort *Port,
				    MUSIC_IndexMap *indices,
				    MUSIC_EventHandler *handleEvent,
				    double accLatency,
				    int maxBuffered)
{
  MUSIC::EventInputPort* cxxPort = (MUSIC::EventInputPort *) Port;
  MUSIC::IndexMap* cxxIndices = (MUSIC::IndexMap *) indices;
  MUSIC::EventHandlerLocalIndexProxy* cxxHandleEvent =
    cxxPort->allocEventHandlerLocalIndexProxy (handleEvent);
  cxxPort->map (cxxIndices, cxxHandleEvent, accLatency, maxBuffered);
}


void
MUSIC_MessageOutputPort_map_no_handler (MUSIC_MessageOutputPort *Port)
{
  MUSIC::MessageOutputPort* cxxPort = (MUSIC::MessageOutputPort *) Port;
  cxxPort->map ();
}


void
MUSIC_MessageOutputPort_map (MUSIC_MessageOutputPort *Port,
			     int maxBuffered)
{
  MUSIC::MessageOutputPort* cxxPort = (MUSIC::MessageOutputPort *) Port;
  cxxPort->map (maxBuffered);
}


typedef void MUSIC_MessageHandler (double t, void *msg, size_t size);

void
MUSIC_MessageInputPort_map (MUSIC_MessageInputPort *Port,
			    MUSIC_MessageHandler *handleMessage,
			    double accLatency,
			    int maxBuffered)
{
  MUSIC::MessageInputPort* cxxPort = (MUSIC::MessageInputPort *) Port;
  MUSIC::MessageHandlerProxy* cxxHandleMessage =
    cxxPort->allocMessageHandlerProxy (handleMessage);
  cxxPort->map (cxxHandleMessage, accLatency, maxBuffered);
}


/* Index maps */

MUSIC_PermutationIndex *
MUSIC_createPermutationIndex (int *indices,
			      int size)
{
  void* data = static_cast<void*> (indices);
  return (MUSIC_PermutationIndex *)
    new MUSIC::PermutationIndex (static_cast<MUSIC::GlobalIndex*> (data),
				  size);
}


void
MUSIC_destroyPermutationIndex (MUSIC_PermutationIndex *Index)
{
  delete (MUSIC::PermutationIndex *) Index;
}


MUSIC_LinearIndex *
MUSIC_createLinearIndex (int baseIndex,
			 int size)
{
  return (MUSIC_LinearIndex *) new MUSIC::LinearIndex (baseIndex, size);
}


void
MUSIC_destroyLinearIndex (MUSIC_LinearIndex *Index)
{
  delete (MUSIC::LinearIndex *) Index;
}


/* Data maps */

/* Exception: The map argument can take any type of index map. */

MUSIC_ArrayData *
MUSIC_createArrayData (void *buffer,
		       MPI_Datatype type,
		       void *map)
{
  MUSIC::IndexMap* cxxMap = (MUSIC::IndexMap *) map;
  return (MUSIC_ArrayData *) new MUSIC::ArrayData (buffer, type, cxxMap);
}


/* Exception: MUSIC_createLinearArrayData corresponds to
   c++ MUSIC::ArrayData::ArrayData (..., ..., ..., ...) */

MUSIC_ArrayData *
MUSIC_createLinearArrayData (void *buffer,
			     MPI_Datatype type,
			     int baseIndex,
			     int size)
{
  return (MUSIC_ArrayData *) new MUSIC::ArrayData (buffer,
						   type,
						   baseIndex,
						   size);
}


void
MUSIC_destroyArrayData (MUSIC_ArrayData *ArrayData)
{
  delete (MUSIC::ArrayData *) ArrayData;
}


/* Configuration variables */

/* Exceptions: Result is char *
   Extra maxlen argument prevents buffer overflow.
   Result is terminated by \0 unless longer than maxlen - 1 */

int
MUSIC_configString (MUSIC_Setup *setup,
		    char *name,
		    char *result,
		    size_t maxlen)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  std::string cxxResult;
  int res = cxxSetup->config (string (name), &cxxResult);
  strncpy(result, cxxResult.c_str (), maxlen);
  return res;
}


int
MUSIC_configInt (MUSIC_Setup *setup, char *name, int *result)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return cxxSetup->config (string(name), result);
}


int
MUSIC_configDouble (MUSIC_Setup *setup, char *name, double *result)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return cxxSetup->config (string(name), result);
}


/* Runtime */

MUSIC_Runtime *
MUSIC_createRuntime (MUSIC_Setup *setup, double h)
{
  MUSIC::Setup* cxxSetup = (MUSIC::Setup *) setup;
  return (MUSIC_Runtime *) new MUSIC::Runtime (cxxSetup, h);
}


void
MUSIC_tick (MUSIC_Runtime *runtime)
{
  MUSIC::Runtime* cxxRuntime = (MUSIC::Runtime *) runtime;
  cxxRuntime->tick ();
}


double
MUSIC_time (MUSIC_Runtime *runtime)
{
  MUSIC::Runtime* cxxRuntime = (MUSIC::Runtime *) runtime;
  return cxxRuntime->time ();
}


/* Finalization */

void
MUSIC_destroyRuntime (MUSIC_Runtime *runtime)
{
  MUSIC::Runtime* cxxRuntime = (MUSIC::Runtime *) runtime;
  delete cxxRuntime;
}

}
#endif
