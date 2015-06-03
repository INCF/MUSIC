#ifndef MUSIC_C_H

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
#include "music/music-config.hh"

#if MUSIC_USE_MPI
#include <mpi.h>

#if MUSIC_HAVE_SIZE_T
#include <sys/types.h>
#else
typedef int size_t;
#endif

/* Setup */

typedef struct MUSIC_Setup MUSIC_Setup;

MUSIC_Setup *MUSIC_createSetup (int *argc, char ***argv);
MUSIC_Setup *MUSIC_createSetupThread (int *argc,
				      char ***argv,
				      int required,
				      int* provided);

/* Communicators */

#ifndef BUILDING_MUSIC_LIBRARY
MPI_Comm MUSIC_setupCommunicator (MUSIC_Setup *setup);
#endif

/* Ports */

typedef struct MUSIC_ContOutputPort MUSIC_ContOutputPort;
typedef struct MUSIC_ContInputPort MUSIC_ContInputPort;
typedef struct MUSIC_EventOutputPort MUSIC_EventOutputPort;
typedef struct MUSIC_EventInputPort MUSIC_EventInputPort;
typedef struct MUSIC_MessageOutputPort MUSIC_MessageOutputPort;
typedef struct MUSIC_MessageInputPort MUSIC_MessageInputPort;

MUSIC_ContOutputPort *MUSIC_publishContOutput (MUSIC_Setup *setup, char *id);
MUSIC_ContInputPort *MUSIC_publishContInput (MUSIC_Setup *setup, char *id);
MUSIC_EventOutputPort *MUSIC_publishEventOutput (MUSIC_Setup *setup, char *id);
MUSIC_EventInputPort *MUSIC_publishEventInput (MUSIC_Setup *setup, char *id);
MUSIC_MessageOutputPort *MUSIC_publishMessageOutput (MUSIC_Setup *setup, char *id);
MUSIC_MessageInputPort *MUSIC_publishMessageInput (MUSIC_Setup *setup, char *id);

/* remedius
 *  Free of the ports is done by the Runtime instance
 */
/*void MUSIC_destroyContOutput (MUSIC_ContOutputPort* port);
void MUSIC_destroyContInput (MUSIC_ContInputPort* port);
void MUSIC_destroyEventOutput (MUSIC_EventOutputPort* port);
void MUSIC_destroyEventInput (MUSIC_EventInputPort* port);
void MUSIC_destroyMessageOutput (MUSIC_MessageOutputPort* port);
void MUSIC_destroyMessageInput (MUSIC_MessageInputPort* port);
*/
/* General port methods */

int MUSIC_ContOutputPort_isConnected (MUSIC_ContOutputPort *port);
int MUSIC_ContInputPort_isConnected (MUSIC_ContInputPort *port);
int MUSIC_EventOutputPort_isConnected (MUSIC_EventOutputPort *port);
int MUSIC_EventInputPort_isConnected (MUSIC_EventInputPort *port);
int MUSIC_MessageOutputPort_isConnected (MUSIC_MessageOutputPort *port);
int MUSIC_MessageInputPort_isConnected (MUSIC_MessageInputPort *port);
int MUSIC_ContOutputPort_hasWidth (MUSIC_ContOutputPort *port);
int MUSIC_ContInputPort_hasWidth (MUSIC_ContInputPort *port);
int MUSIC_EventOutputPort_hasWidth (MUSIC_EventOutputPort *port);
int MUSIC_EventInputPort_hasWidth (MUSIC_EventInputPort *port);
int MUSIC_ContOutputPort_width (MUSIC_ContOutputPort *port);
int MUSIC_ContInputPort_width (MUSIC_ContInputPort *port);
int MUSIC_EventOutputPort_width (MUSIC_EventOutputPort *port);
int MUSIC_EventInputPort_width (MUSIC_EventInputPort *port);

/* Mapping */

/* Data maps */

typedef struct MUSIC_DataMap MUSIC_DataMap;
typedef struct MUSIC_ContData MUSIC_ContData;
typedef struct MUSIC_ArrayData MUSIC_ArrayData;

/* Index maps */

typedef struct MUSIC_IndexMap MUSIC_IndexMap;
typedef struct MUSIC_PermutationIndex MUSIC_PermutationIndex;
typedef struct MUSIC_LinearIndex MUSIC_LinearIndex;


/* No arguments are optional. */

void MUSIC_ContOutputPort_map (MUSIC_ContOutputPort *port,
			       MUSIC_ContData *dmap,
			       int maxBuffered);

void MUSIC_ContInputPort_map (MUSIC_ContInputPort *port,
			      MUSIC_ContData *dmap,
			      double delay,
			      int maxBuffered,
			      int interpolate);

void MUSIC_EventOutputPort_mapGlobalIndex (MUSIC_EventOutputPort *Port,
					   MUSIC_IndexMap *indices,
					   int maxBuffered);

void MUSIC_EventOutputPort_mapLocalIndex (MUSIC_EventOutputPort *Port,
					  MUSIC_IndexMap *indices,
					  int maxBuffered);

typedef void MUSIC_EventHandler (double t, int id);

void MUSIC_EventInputPort_mapGlobalIndex (MUSIC_EventInputPort *port,
					  MUSIC_IndexMap *indices,
					  MUSIC_EventHandler *handleEvent,
					  double accLatency,
					  int maxBuffered);

void MUSIC_EventInputPort_mapLocalIndex (MUSIC_EventInputPort *port,
					 MUSIC_IndexMap *indices,
					 MUSIC_EventHandler *handleEvent,
					 double accLatency,
					 int maxBuffered);

void MUSIC_MessageOutputPort_map_no_handler (MUSIC_MessageOutputPort *port);

void MUSIC_MessageOutputPort_map (MUSIC_MessageOutputPort *port,
				  int maxBuffered);

typedef void MUSIC_MessageHandler (double t, void *msg, size_t size);

void MUSIC_MessageInputPort_map (MUSIC_MessageInputPort *port,
				 MUSIC_MessageHandler *handleMessage,
				 double accLatency,
				 int maxBuffered);

/* Index maps */

MUSIC_PermutationIndex *MUSIC_createPermutationIndex (int *indices,
						      int size);

void MUSIC_destroyPermutationIndex (MUSIC_PermutationIndex *Index);

MUSIC_LinearIndex *MUSIC_createLinearIndex (int baseIndex,
					    int size);

void MUSIC_destroyLinearIndex (MUSIC_LinearIndex *Index);

/* Exception: The map argument can take any type of index map. */

MUSIC_ArrayData *MUSIC_createArrayData (void *buffer,
					MPI_Datatype type,
					void *map);

/* Exception: MUSIC_createLinearArrayData corresponds to
   c++ MUSIC::ArrayData::ArrayData (..., ..., ..., ...) */

MUSIC_ArrayData *MUSIC_createLinearArrayData (void *buffer,
					      MPI_Datatype type,
					      int baseIndex,
					      int size);

void MUSIC_destroyArrayData (MUSIC_ArrayData *arrayData);

/* Configuration variables */

/* Exceptions: Result is char *
   Extra maxlen argument prevents buffer overflow.
   Result is terminated by \0 unless longer than maxlen - 1 */

int MUSIC_configString (MUSIC_Setup *setup,
			char *name,
			char *result,
			size_t maxlen);

int MUSIC_configInt (MUSIC_Setup *setup, char *name, int *result);

int MUSIC_configDouble (MUSIC_Setup *setup, char *name, double *result);

/* Runtime */

typedef struct MUSIC_Runtime MUSIC_Runtime;

MUSIC_Runtime *MUSIC_createRuntime (MUSIC_Setup *setup, double h);

void MUSIC_tick (MUSIC_Runtime *runtime);

double MUSIC_time (MUSIC_Runtime *runtime);

/* Finalization */

void MUSIC_destroyRuntime (MUSIC_Runtime *runtime);
#endif
#define MUSIC_C_H
#endif /* MUSIC_C_H */
