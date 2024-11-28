
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <string>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/random.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
//#include <boost/log/trivial.hpp>

#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include "v4l2_driver.h"

using namespace boost::interprocess;

#if 0
/* two planes -- one Y, one Cr + Cb interleaved  */
#define V4L2_PIX_FMT_NV12    v4l2_fourcc('N', 'V', '1', '2') /* 12  Y/CbCr 4:2:0  */
#define V4L2_PIX_FMT_NV21    v4l2_fourcc('N', 'V', '2', '1') /* 12  Y/CrCb 4:2:0  */

#define V4L2_PIX_FMT_MJPEG    v4l2_fourcc('M', 'J', 'P', 'G') /* Motion-JPEG   */
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '5') /* H264 with start codes */
#endif


uint32_t loop = 0;

void print_affinity()
{
    cpu_set_t mask;
    long nproc, i;

    if (sched_getaffinity(0, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_getaffinity");
        assert(false);
    }
    nproc = sysconf(_SC_NPROCESSORS_ONLN);
    printf("sched_getaffinity = ");
    for (i = 0; i < nproc; i++) {
        printf("%d ", CPU_ISSET(i, &mask));
    }
    printf("\n");
}

void handle_signal(int _signal) 
{

    if (_signal == SIGINT ) {
        printf("SIGINT \n"); 
        loop = 0;
    }
    else if( _signal == SIGTERM){
        printf("SIGTERM \n"); 
    }

}

int main(int argc, char *argv[])
{
    uint32_t seed = 0;
    uint32_t flength = 0;

    struct timeval timestamp;

    const char *device = "/dev/video0";
    int video_fildes;
    uint8_t *shared_mem;
    uint8_t *sptr;

    int ret;
    struct timeval tv = {.tv_sec = 3, .tv_usec = 0};
    fd_set fds;
    struct v4l2_buffer buf;

    std::vector<uint8_t *> index_buffers;

    if(argc != 2)
    {
        std::cerr << "argument Error: " << "you should input videoX " << std::endl;
        return 0;
    }

    device = argv[1]; 
    std::cout << "Device : [ " << device << " ] " << std::endl;

#if 0
    cpu_set_t mask;

    print_affinity();
    CPU_ZERO(&mask);
    CPU_SET(2, &mask);

    if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        //assert(false);
    }
    print_affinity();
#endif

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    //Remove shared memory on construction and destruction
    struct shm_remove
    {
       shm_remove() { shared_memory_object::remove("MySharedMemory"); }
       ~shm_remove(){ shared_memory_object::remove("MySharedMemory"); }
    } remover;

    //Create a shared memory object.
    shared_memory_object shm (create_only, "MySharedMemory", read_write);

    video_fildes = v4l2_open(device);
    if (video_fildes == -1) {
      fprintf(stderr, "can't open %s\n", device);
      exit(-1);
    }

    if (v4l2_querycap(video_fildes, device) == -1) {
      perror("v4l2_querycap");
      return 1;
    }

    // most of devices support YUYV422 packed.
    //if (v4l2_sfmt(video_fildes, V4L2_PIX_FMT_YUYV) == -1) 
    //if (v4l2_sfmt(video_fildes, V4L2_PIX_FMT_MJPEG) == -1) 
    if (v4l2_sfmt(video_fildes, V4L2_PIX_FMT_NV12) == -1) 
    {
      perror("v4l2_sfmt");
      return 1;
    }
 
    if (v4l2_gfmt(video_fildes) == -1) {
      perror("v4l2_gfmt");
      return 1;
    }
 
    if (v4l2_sfps(video_fildes, 12) == -1) { // no fatal error
      perror("v4l2_sfps");
    }
 
    if (v4l2_mmap(video_fildes) == -1) {
      perror("v4l2_mmap");
      return 1;
    }

    //Set size
    //shm.truncate(v4l2_ubuffers[0].length+1024);
    shm.truncate((v4l2_ubuffers[0].length*8) + 16);
    flength = v4l2_ubuffers[0].length;
 
    //Map the whole shared memory in this process
    mapped_region region(shm, read_write);

    shared_mem = static_cast<uint8_t *>(region.get_address());

    index_buffers.push_back(&shared_mem[0]+8);
    index_buffers.push_back(&shared_mem[flength]+8);
    index_buffers.push_back(&shared_mem[flength*2]+8);
    index_buffers.push_back(&shared_mem[flength*3]+8);
    index_buffers.push_back(&shared_mem[flength*4]+8);
    index_buffers.push_back(&shared_mem[flength*5]+8);
    index_buffers.push_back(&shared_mem[flength*6]+8);
    index_buffers.push_back(&shared_mem[flength*7]+8);

    std::cout << "Stream On ... " << std::endl;
    if (v4l2_streamon(video_fildes) == -1) {
      perror("v4l2_streamon");
    }

    tv.tv_sec = 3;
    tv.tv_usec = 0;
    seed = 0;

    loop = 1;

    while(loop)
    {
        FD_ZERO(&fds);
        FD_SET(video_fildes, &fds);
       
        ret = select(video_fildes+1, &fds, NULL, NULL, &tv);
        if (-1 == ret) {
            fprintf(stderr, "select error\n");
            return 0;
        } else if (0 == ret) {
            //fprintf(stderr, "timeout waiting for frame\n");
            usleep(30*1000);
            continue;
        }
       
        if (FD_ISSET(video_fildes, &fds)) {
       
          memset(&buf, 0, sizeof(buf));
          buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory = V4L2_MEMORY_MMAP;
       
          if (-1 == ioctl(video_fildes, VIDIOC_DQBUF, &buf)) {
            fprintf(stderr, "VIDIOC_DQBUF failure\n");
            return 1;
          }

          flength = buf.bytesused;
          timestamp = buf.timestamp;

          //std::memcpy( &shared_mem[sizeof(uint32_t)], &flength, sizeof(uint32_t) );
          //std::memcpy( &shared_mem[sizeof(uint32_t)*2], &timestamp, sizeof(struct timeval) );
          //std::memcpy( &shared_mem[sizeof(uint32_t)*2+sizeof(struct timeval)], v4l2_ubuffers[buf.index].start, flength );

          sptr = index_buffers[buf.index];

          std::memcpy( &sptr[sizeof(uint32_t)], &flength, sizeof(uint32_t) );
          std::memcpy( &sptr[sizeof(uint32_t)*2], &timestamp, sizeof(struct timeval) );
          std::memcpy( &sptr[sizeof(uint32_t)*2+sizeof(struct timeval)], v4l2_ubuffers[buf.index].start, flength );

          std::memcpy( &shared_mem[4], &buf.index, sizeof(uint32_t) );
          std::memcpy( shared_mem, &seed, sizeof(uint32_t) );

#ifdef DEBUG
          //if(seed % 300 == 0){
          //    printf("id [%d] byteused [ %d ], timestamp [ %ld.%06ld ] \n", 
          //                      seed, buf.bytesused, timestamp.tv_sec, timestamp.tv_usec);
          //}
          printf("deque buffer [%d] byteused [ %d ] (buffer length: %d) \n", 
                            buf.index, buf.bytesused, v4l2_ubuffers[buf.index].length);
          printf("seed [%d] seed index [ %d ] \n\n", seed % 8, seed); 
#endif
          seed++;

          if(seed == 65536)
              seed = 1;

          buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory = V4L2_MEMORY_MMAP;
          if (-1 == ioctl(video_fildes, VIDIOC_QBUF, &buf)) {
            fprintf(stderr, "VIDIOC_QBUF failure\n");
            return 0;
          }

//#ifdef DEBUG
//          printf("queue buffer %d\n", buf.index);
//#endif
        }

        //seed = static_cast<uint32_t>(time(0));
        //lagged_fibonacci607(seed);
        //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }

    if (v4l2_streamoff(video_fildes) == -1) {
      perror("v4l2_streamoff");
    }
 
    if (v4l2_munmap() == -1) {
      perror("v4l2_munmap");
    }
 
    if (v4l2_close(video_fildes) == -1) {
      perror("v4l2_close");
    }

    struct shm_remove_1
    {
       shm_remove_1() { shared_memory_object::remove("MySharedMemory"); }
       ~shm_remove_1(){ shared_memory_object::remove("MySharedMemory"); }
    } remover_1;

    return 0;
}

