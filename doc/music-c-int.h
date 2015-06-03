/* Setup */

MUSIC_Setup *MUSIC_createSetup (int *argc, char ***argv);

/* Communicators */

MPI_Intracomm MUSIC_Setup_communicator (MUSIC_setup *setup);

/* Port creation */

MUSIC_ContOutputPort *MUSIC_publishContOutput (MUSIC_setup *setup,
					       char *id);
MUSIC_ContInputPort *MUSIC_publishContInput (MUSIC_setup *setup,
					     char *id);
MUSIC_EventOutputPort *MUSIC_publishEventOutput (MUSIC_setup *setup,
						 char *id);
MUSIC_EventInputPort *MUSIC_publishEventInput (MUSIC_setup *setup,
					       char *id);
MUSIC_MessageOutputPort *MUSIC_publishMessageOutput (MUSIC_setup *setup,
						     char *id);
MUSIC_MessageInputPort *MUSIC_publishMessageInput (MUSIC_setup *setup,
						   char *id);

void MUSIC_destroyContOutput (MUSIC_ContOutputPort* port);
void MUSIC_destroyContInput (MUSIC_ContInputPort* port);
void MUSIC_destroyEventOutput (MUSIC_EventOutputPort* port);
void MUSIC_destroyEventInput (MUSIC_EventInputPort* port);
void MUSIC_destroyMessageOutput (MUSIC_MessageOutputPort* port);
void MUSIC_destroyMessageInput (MUSIC_MessageInputPort* port);

/* General port methods */

/* Xxx = Cont | Event
   Ddd = Output | Input */

int MUSIC_XxxDddPort_isConnected (XxxDddPort *port);
int MUSIC_MessageDddPort_isConnected (MessageDddPort *port);
int MUSIC_XxxDddPort_hasWidth (XxxDddPort *port);
int MUSIC_XxxDddPort_width (XxxDddPort *port);

/* Mapping */

/* No arguments are optional. */

void MUSIC_ContOutputPort_map (MUSIC_ContOutputPort *port,
			       MUSIC_ContData *dMap,
			       int maxBuffered);

void MUSIC_ContInputPort_map (MUSIC_ContInputPort *port,
			      MUSIC_ContData *dMap,
			      double delay,
			      int maxBuffered,
			      int interpolate);

void MUSIC_EventOutputPort_map (MUSIC_EventOutputPort *port,
				MUSIC_IndexMap *indices,
				int maxBuffered);

typedef void MUSIC_EventHandler (double t, int id);

void MUSIC_EventInputPort_map (MUSIC_EventInputPort *port,
			       MUSIC_IndexMap *indices,
			       MUSIC_EventHandler *handleEvent,
			       double accLatency,
			       int maxBuffered);

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

void MUSIC_destroyPermutationIndex (MUSIC_PermutationIndex *index);

MUSIC_LinearIndex *MUSIC_createLinearIndex (int baseIndex,
					    int size);

void MUSIC_destroyLinearIndex (MUSIC_LinearIndex *index);

/* Data maps */

/* Exception: The map argument can take any type of index map. */

MUSIC_ArrayData *MUSIC_createArrayData (void *buffer,
					MPI_Datatype type,
					void *map);

/* Exception: MUSIC_createLinearArrayData corresponds to
   c++ music::ArrayData::ArrayData (..., ..., ..., ...) */

MUSIC_ArrayData *MUSIC_createLinearArrayData (void *buffer,
					      MPI_Datatype type,
					      int baseIndex,
					      int size);

void MUSIC_destroyArrayData (MUSIC_ArrayData *arrayData);

/* Configuration variables */

/* Exceptions: Result is char *
   Extra maxlen argument prevents buffer overflow.
   Result is terminated by \0 unless longer than maxlen - 1 */

int MUSIC_config (MUSIC_Setup *setup,
		  char *name,
		  char *result,
		  size_t maxLen);

int MUSIC_config (MUSIC_Setup *setup, char *name, int *result);

int MUSIC_config (MUSIC_Setup *setup, char *name, double *result);

/* Runtime */

MUSIC_Runtime *MUSIC_createRuntime (MUSIC_Setup *setup, double h);

void MUSIC_tick (MUSIC_Runtime *runtime);

double MUSIC_time (MUSIC_Runtime *runtime);

/* Finalization */

void MUSIC_destroyRuntime (MUSIC_Runtime *runtime);
