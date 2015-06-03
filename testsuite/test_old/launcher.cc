#include <stdio.h>
#include <unistd.h>
#include <rts.h>


extern char **environ;


int main(int nargs, char **argv) {
  int err;

  BGLPersonality whoAmI;

  rts_get_personality(&whoAmI, sizeof whoAmI);

  if(whoAmI.zCoord < whoAmI.zSize / 2) {
    printf("%d starts as first\n", whoAmI.zCoord);
    err = execve("/gpfs/scratch/o/orjan/application1", argv, environ);
  } else {
    printf("%d starts as second\n", whoAmI.zCoord);
    err = execve("/gpfs/scratch/o/orjan/application2", argv, environ);
  }

  //Should not get here....
  printf("Note: execve call returned %d\n", err);
  return(-1);
}
