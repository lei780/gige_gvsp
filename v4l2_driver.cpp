#include "v4l2_driver.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

int IMAGE_WIDTH = 1280;
int IMAGE_HEIGHT = 720;
struct v4l2_ubuffer *v4l2_ubuffers;

int v4l2_open(const char *device) {
  struct stat st;
  memset(&st, 0, sizeof(st));
  if (stat(device, &st) == -1) {
    perror("stat");
    return -1;
  }
  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "%s is no character device\n", device);
    return -1;
  } else
    printf("%s is a character device\n", device);
  return open(device, O_RDWR | O_NONBLOCK, 0);
}

int v4l2_close(int fd) { return close(fd); }

int v4l2_querycap(int fd, const char *device) {
  struct v4l2_capability cap;
  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
    printf("Error opening device %s: unable to query device.\n", device);
    return -1;
  } else {
    printf("driver:\t\t%s\n", cap.driver);
    printf("card:\t\t%s\n", cap.card);
    printf("bus_info:\t%s\n", cap.bus_info);
    printf("version:\t%d\n", cap.version);
    printf("capabilities:\t%x\n", cap.capabilities);

    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE) {
      printf("Device %s: supports capture.\n", device);
    }

    if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING) {
      printf("Device %s: supports streaming.\n", device);
    }
  }

  // emu all support fmt
  struct v4l2_fmtdesc fmtdesc;
  fmtdesc.index = 0;
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  printf("\033[31mSupport format:\n\033[0m");
  while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
    printf("\033[31m\t%d.%s\n\033[0m", fmtdesc.index + 1, fmtdesc.description);
    fmtdesc.index++;
  }
  return 0;
}

// set format
int v4l2_sfmt(int fd, uint32_t pfmt) {
  // set fmt
  struct v4l2_format fmt;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.pixelformat = pfmt;
  fmt.fmt.pix.height = IMAGE_HEIGHT;
  fmt.fmt.pix.width = IMAGE_WIDTH;
  //fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;

  if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
    fprintf(stderr, "Unable to set format\n");
    return -1;
  }
  return 0;
}

// get format
int v4l2_gfmt(int fd) {
  // set fmt
  struct v4l2_format fmt;
  if (ioctl(fd, VIDIOC_G_FMT, &fmt) == -1) {
    fprintf(stderr, "Unable to get format\n");
    return -1;
  }
  printf("\033[33mpix.pixelformat:\t%c%c%c%c\n\033[0m",
         fmt.fmt.pix.pixelformat & 0xFF, (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
         (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
         (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
  printf("pix.height:\t\t%d\n", fmt.fmt.pix.height);
  printf("pix.width:\t\t%d\n", fmt.fmt.pix.width);
  printf("pix.field:\t\t%d\n", fmt.fmt.pix.field);
  return 0;
}

int v4l2_sfps(int fd, int fps) 
{
  struct v4l2_streamparm setfps;

  memset(&setfps, 0, sizeof(setfps));

  setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  setfps.parm.capture.timeperframe.numerator = 1;
  setfps.parm.capture.timeperframe.denominator = fps;

  if (ioctl(fd, VIDIOC_S_PARM, &setfps) == -1) 
  {
    // no fatal error ,just put err msg
    fprintf(stderr, "Unable to set framerate\n");
    return -1;
  }

  return 0;
}

int v4l2_mmap(int fd) 
{
  // request for 4 buffers
  struct v4l2_requestbuffers req;
  struct v4l2_buffer buf;
  unsigned int n_buffers;
  int ret = -1;

  req.count = BUF_NUM;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  ret = ioctl(fd, VIDIOC_REQBUFS, &req);
  if (ret < 0) 
  {
    fprintf(stderr, "request for buffers error\n");
    return ret;
  }

  // mmap for buffers
  v4l2_ubuffers = (struct v4l2_ubuffer *)malloc(req.count * sizeof(struct v4l2_ubuffer));
  if (v4l2_ubuffers == NULL) 
  {
    fprintf(stderr, "Out of memory\n");
    return -1;
  }

  for (n_buffers = 0; n_buffers < req.count; n_buffers++) 
  {
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    /* query buffers */
    ret = ioctl(fd, VIDIOC_QUERYBUF, &buf);
    if (ret < 0) 
    {
      fprintf(stderr, "query buffer error\n");
      return ret;
    }

    /* map 4 buffers in driver space to usersapce */
    v4l2_ubuffers[n_buffers].length = buf.length;
    v4l2_ubuffers[n_buffers].start = mmap(NULL, buf.length, 
                                          PROT_READ|PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
#ifdef DEBUG
    printf("buffer offset:%d\tlength:%d\n", buf.m.offset, buf.length);
#endif

    /**
      *  output:
      *  buffer offset:0	length:614400
      *  buffer offset:614400	length:614400
      *  buffer offset:1228800	length:614400
      *  buffer offset:1843200	length:614400
      *
      *  explanation：saved in YUV422 format，a pixel needs 2 byte storage in
      *  average，as our image size is 640*480. 640*480*2=614400
    */
    if (v4l2_ubuffers[n_buffers].start == MAP_FAILED) 
    {
      fprintf(stderr, "buffer map error %u\n", n_buffers);
      return -1;
    }
  }

  return ret;
}

int v4l2_mmap_shared_memory(int fd, char *shared_buffer) 
{
  // request for 4 buffers
  struct v4l2_requestbuffers req;
  struct v4l2_buffer buf;
  unsigned int n_buffers;
  int ret = -1;

  req.count = BUF_NUM;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  ret = ioctl(fd, VIDIOC_REQBUFS, &req);
  if (ret < 0) 
  {
    fprintf(stderr, "request for buffers error\n");
    return ret;
  }

  // mmap for buffers
  v4l2_ubuffers = (struct v4l2_ubuffer *)malloc(req.count * sizeof(struct v4l2_ubuffer));
  if (v4l2_ubuffers == NULL) 
  {
    fprintf(stderr, "Out of memory\n");
    return -1;
  }

  for (n_buffers = 0; n_buffers < req.count; n_buffers++) 
  {
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    /* query buffers */
    ret = ioctl(fd, VIDIOC_QUERYBUF, &buf);
    if (ret < 0) 
    {
      fprintf(stderr, "query buffer error\n");
      return ret;
    }

    /* map 4 buffers in driver space to usersapce */
    v4l2_ubuffers[n_buffers].length = buf.length;
    v4l2_ubuffers[n_buffers].start = mmap(NULL, buf.length, 
                                          PROT_READ|PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
#ifdef DEBUG
    printf("buffer offset:%d\tlength:%d\n", buf.m.offset, buf.length);
#endif

    /**
      *  output:
      *  buffer offset:0	length:614400
      *  buffer offset:614400	length:614400
      *  buffer offset:1228800	length:614400
      *  buffer offset:1843200	length:614400
      *
      *  explanation：saved in YUV422 format，a pixel needs 2 byte storage in
      *  average，as our image size is 640*480. 640*480*2=614400
    */
    if (v4l2_ubuffers[n_buffers].start == MAP_FAILED) 
    {
      fprintf(stderr, "buffer map error %u\n", n_buffers);
      return -1;
    }
  }

  return ret;
}

int v4l2_munmap() 
{
  int i;

  for (i = 0; i < BUF_NUM; i++) 
  {
    if (munmap(v4l2_ubuffers[i].start, v4l2_ubuffers[i].length) == -1) 
    {
      fprintf(stderr, "munmap failure %d\n", i);
      return -1;
    }
  }

  return 0;
}

int v4l2_streamon(int fd) 
{
  // queue in the four buffers allocated by VIDIOC_REQBUFS, pretty like water
  // filling a bottle in turn
  struct v4l2_buffer buf;
  unsigned int n_buffers;

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  for (n_buffers = 0; n_buffers < BUF_NUM; n_buffers++) 
  {
    buf.index = n_buffers;
    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
      fprintf(stderr, "queue buffer failed\n");
      return -1;
    }
  }

  if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
    fprintf(stderr, "stream on failed\n");
    return -1;
  }
  return 0;
}

int v4l2_streamoff(int fd) 
{
  int ret = -1;
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  ret = ioctl(fd, VIDIOC_STREAMOFF, &type);
  if (ret < 0) {
    fprintf(stderr, "stream off failed\n");
    return ret;
  }

  printf("stream is off\n");
  return ret;
}

