#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

void setup_rt(void){

    printf("Setting up real-time environment...\n");
    
    mlockall(MCL_CURRENT | MCL_FUTURE);

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    struct sched_param sp = { .sched_priority = 90 };
    sched_setscheduler(0, SCHED_FIFO, &sp);

    // Suppress C-state latency
    int lat_fd = open("/dev/cpu_dma_latency", O_WRONLY);
    if (lat_fd >= 0) {
        static int32_t zero = 0;
        write(lat_fd, &zero, sizeof(zero));
    }

}