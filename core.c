
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <time.h>

int main(int argc, char **argv)
{
  struct timeval timestamp;
  int i, online=0;

  printf("argc = %d \n", argc);

  ulong ncores = sysconf(_SC_NPROCESSORS_CONF);
  cpu_set_t *setp = CPU_ALLOC(ncores);
  ulong setsz = CPU_ALLOC_SIZE(ncores);

  CPU_ZERO_S(setsz, setp);

  if (sched_getaffinity(0, setsz, setp) == -1) {
    perror("sched_getaffinity(2) failed");
    exit(errno);
  }

  for (i=0; i < CPU_COUNT_S(setsz, setp); i++) {
    if (CPU_ISSET_S(i, setsz, setp))
      online++;
  }
	
  printf("%ld cores configured, %d cpus allowed in affinity mask\n", ncores, online);
  CPU_FREE(setp);

  printf("sizeof timeval : %ld \n", sizeof(timestamp));

}

