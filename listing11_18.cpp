
#include <iostream>
#include <thread>
#include <queue>
#include <vector>
#include <list>
#include <functional>

#include <csignal>
#include <cstdio>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/bind/bind.hpp>

#include <boost/atomic.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

//#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
//#include <sched.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/videodev2.h>

#include "v4l2_driver.h"
#include "gvcp_header.h"

namespace asio = boost::asio;
namespace sys = boost::system;
using namespace boost::placeholders;

const size_t MAXBUF = 256;

char g_camera_memory[1024];

#define ARV_FAKE_CAMERA_MEMORY_SIZE 0x10000

#define ARV_GVSP_PACKET_EXTENDED_ID_MODE_MASK   0x80
#define ARV_GVSP_PACKET_ID_MASK         0x00ffffff
#define ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_MASK 0x7f000000
#define ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_POS  24
#define ARV_GVSP_PACKET_INFOS_N_PARTS_MASK      0x000000ff

typedef enum {
    ARV_GVSP_CONTENT_TYPE_LEADER =      0x01,
    ARV_GVSP_CONTENT_TYPE_TRAILER =     0x02,
    ARV_GVSP_CONTENT_TYPE_PAYLOAD =     0x03,
    ARV_GVSP_CONTENT_TYPE_ALL_IN =      0x04,
    ARV_GVSP_CONTENT_TYPE_H264 =        0x05,
    ARV_GVSP_CONTENT_TYPE_MULTIZONE =   0x06,
    ARV_GVSP_CONTENT_TYPE_MULTIPART =   0x07,
    ARV_GVSP_CONTENT_TYPE_GENDC =       0x08
} ArvGvspContentType;


typedef enum {
    ARV_BUFFER_PAYLOAD_TYPE_UNKNOWN =               -1,
    ARV_BUFFER_PAYLOAD_TYPE_NO_DATA =       0x0000,
    ARV_BUFFER_PAYLOAD_TYPE_IMAGE =         0x0001,
    ARV_BUFFER_PAYLOAD_TYPE_RAWDATA =       0x0002,
    ARV_BUFFER_PAYLOAD_TYPE_FILE =          0x0003,
    ARV_BUFFER_PAYLOAD_TYPE_CHUNK_DATA =        0x0004,
    ARV_BUFFER_PAYLOAD_TYPE_EXTENDED_CHUNK_DATA =   0x0005, /* Deprecated */
    ARV_BUFFER_PAYLOAD_TYPE_JPEG =          0x0006,
    ARV_BUFFER_PAYLOAD_TYPE_JPEG2000 =      0x0007,
    ARV_BUFFER_PAYLOAD_TYPE_H264 =          0x0008,
    ARV_BUFFER_PAYLOAD_TYPE_MULTIZONE_IMAGE =   0x0009,
    ARV_BUFFER_PAYLOAD_TYPE_MULTIPART =             0x000a,
    ARV_BUFFER_PAYLOAD_TYPE_GENDC_CONTAINER =       0x000b,
    ARV_BUFFER_PAYLOAD_TYPE_GENDC_COMPONENT_DATA =  0x000c
} ArvBufferPayloadType;


typedef struct {
    uint16_t packet_type;
    uint8_t header[];
} __attribute__((packed)) ArvGvspPacket;


typedef struct {
    uint16_t frame_id;
    uint32_t packet_infos;
    uint8_t data[];
} __attribute__((packed)) ArvGvspHeader;


typedef struct {
    uint16_t flags;
    uint32_t packet_infos;
    uint64_t frame_id;
    uint32_t packet_id;
    uint8_t data[];
} __attribute__((packed)) ArvGvspExtendedHeader;


typedef struct {
    uint32_t payload_type;
    uint32_t data0;
} __attribute__((packed))  ArvGvspTrailer;


typedef struct {
    uint32_t pixel_format;
    uint32_t width;
    uint32_t height;
    uint32_t x_offset;
    uint32_t y_offset;
    uint16_t x_padding;
    uint16_t y_padding;
} __attribute__((packed)) ArvGvspImageInfos;


typedef struct {
    uint16_t flags;
    uint16_t payload_type;
    uint32_t timestamp_high;
    uint32_t timestamp_low;
    ArvGvspImageInfos infos;
} __attribute__((packed)) ArvGvspImageLeader;


/**
 * ArvGvcpHeader:
 * @packet_type: a #ArvGvcpPacketType identifier
 * @packet_flags: set of packet flags
 * @command: a #ArvGvcpCommand identifier
 * @size: data size
 * @id: packet identifier
 *
 * GVCP packet header structure.
 */

typedef struct {
    uint8_t packet_type;
    uint8_t packet_flags;
    uint16_t command;
    uint16_t size;
    uint16_t id;
} __attribute__((packed)) ArvGvcpHeader;

/**
 * ArvGvcpPacket:
 * @header: packet header
 * @data: variable size byte array
 *
 * GVCP packet structure.
 */

typedef struct {
    ArvGvcpHeader header;
    unsigned char data[];
} __attribute__((packed)) ArvGvcpPacket;

//std::queue< std::vector<ArvGvspPacket *> > frames;


class UDPAsyncServer 
{
public:
  UDPAsyncServer(asio::io_service& service, unsigned short port) 
     : socket(service, 
          asio::ip::udp::endpoint(asio::ip::udp::v4(), port))
  {  
    init_done = false;
    waitForReceive(); 

    m_transfer_packet_length = 1480;
  }

  ~UDPAsyncServer() 
  { 
      stopDevice();

      //std::cout << __FUNCTION__ << ": Exec.. " << std::endl;
#if BOOST_LOG_DYN_LINK == 1
      BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Enter ..." << std::endl; 
#endif

    if(init_done){
      if (v4l2_munmap() == -1) {
        BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << ": Error Unmap ..." << std::endl; 
      }

      if (v4l2_close(m_video_fd) == -1) {
        BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << ": Error Close ..." << std::endl; 
      }
    }

    loop = 0;
    if(frame_proc.joinable()){
        BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Join Wait ..." ;
        frame_proc.join();
    }

    socket.cancel();
    BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Leave ..." ;
  }

  void waitForReceive() 
  {
#if 0
      socket.async_receive_from( 
           asio::buffer(buffer, MAXBUF), 
           remote_peer,
           boost::bind(&UDPAsyncServer::DataReceive, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred) );
#endif

      socket.async_receive_from( 
           asio::buffer(buffer, MAXBUF), 
           remote_peer,
           boost::bind(&UDPAsyncServer::DataReceive, this, _1, _2) );
  }

  void send_complete (const sys::error_code& ec, size_t sz) 
  {
//#if BOOST_LOG_DYN_LINK == 1
//      BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Exec..." ;
//#endif
  }

  void frame_udp_send (char *pdata, unsigned int length) 
  {
      //BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Exec..." ;
      //std::cout << "Send to " << remote_peer << '\n';

      socket.async_send_to(
          asio::buffer(pdata, length),
          remote_peer,
          boost::bind(&UDPAsyncServer::send_complete, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred) );
  }

  void DataReceive (const sys::error_code& ec, size_t sz) 
  {
      //const char *msg = "hello from server";
      if( ec == boost::asio::error::operation_aborted ) 
      {
          std::cout << __FUNCTION__ << ": Aborted "  << '\n';
          return;
      }

      std::cout << "Received: [" << buffer << "] " << remote_peer << '\n';

      waitForReceive() ;
      //socket.async_send_to(
      //    asio::buffer(msg, strlen(msg)),
      //    remote_peer,
      //    boost::bind(&UDPAsyncServer::send_complete, this,
      //          boost::asio::placeholders::error,
      //          boost::asio::placeholders::bytes_transferred) );

      //startDevice();
  }


  bool gvsp_packet_has_extended_ids (const ArvGvspPacket *packet)
  {
    return (packet->header[2] & ARV_GVSP_PACKET_EXTENDED_ID_MODE_MASK) != 0;
  }


  void * gvsp_packet_get_data (const ArvGvspPacket *packet)
  {
    if (gvsp_packet_has_extended_ids (packet)) {
        ArvGvspExtendedHeader *header = (ArvGvspExtendedHeader *) &packet->header;

        return &header->data;
    } else {
        ArvGvspHeader *header = (ArvGvspHeader *) &packet->header;

        return &header->data;
    }
  }


  ArvGvspPacket * gvsp_packet_new (ArvGvspContentType content_type,
             uint16_t frame_id, uint32_t packet_id, size_t data_size, void *buffer, size_t *buffer_size)
  {
     ArvGvspPacket *packet;
     ArvGvspHeader *header;
     size_t packet_size;

     packet_size = sizeof (ArvGvspPacket) + sizeof (ArvGvspHeader) + data_size;
     if (packet_size == 0 || (buffer != NULL && (buffer_size == NULL || packet_size > *buffer_size)))
         return NULL;

     if (buffer_size != NULL)
         *buffer_size = packet_size;

     if (buffer != NULL){
         packet = (ArvGvspPacket *)buffer;
     }
     else{
         packet = (ArvGvspPacket *)malloc(packet_size);
         std::cout << __FUNCTION__ << ": alloc " << std::endl;
     }

     packet->packet_type = 0;

     //header = (void *) &packet->header;
     header = (ArvGvspHeader *)packet->header;
     header->frame_id = htons (frame_id);
     header->packet_infos = htonl ((packet_id & ARV_GVSP_PACKET_ID_MASK) |
                     ((content_type << ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_POS) &
                      ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_MASK));

     return packet;
  }


  int initDevice(const char *dev_name)
  {
    m_video_fd = v4l2_open(dev_name);
    if (m_video_fd == -1) {
      //fprintf(stderr, "can't open %s\n", dev_name);
      BOOST_LOG_TRIVIAL(error) << dev_name << ": Can't Open .." ;
      return m_video_fd;
    }

    if (v4l2_querycap(m_video_fd, dev_name) == -1) {
      BOOST_LOG_TRIVIAL(info) << ": Query ..." ;
      return -1;
    }

    // most of devices support YUYV422 packed.
    ///if (v4l2_sfmt(m_video_fd, V4L2_PIX_FMT_NV12) == -1)
    //if (v4l2_sfmt(m_video_fd, V4L2_PIX_FMT_MJPEG) == -1)
    if (v4l2_sfmt(m_video_fd, V4L2_PIX_FMT_YUYV) == -1)
    {
      BOOST_LOG_TRIVIAL(error) << "Error Set Format ..." << std::endl; 
      return -1;
    }

    //if (v4l2_gfmt(video_fildes) == -1) {
    //  perror("v4l2_gfmt");
    //  return 1;
    //}

    if (v4l2_sfps(m_video_fd, 30) == -1) { // no fatal error
      //perror("v4l2_sfps");
      BOOST_LOG_TRIVIAL(error) << "Error Set Frame rate ..." << std::endl; 
      return -1;
    }

    if (v4l2_mmap(m_video_fd) == -1) {
      //perror("v4l2_mmap");
      BOOST_LOG_TRIVIAL(error) << "Error Set MMAP ..." << std::endl; 
      return -1;
    }

    init_done = true;
    return m_video_fd;
  }

#if 1
  void videoframe_polling()
  {
    fd_set fds;
    struct timeval tv = {.tv_sec = 3, .tv_usec = 0};
    struct v4l2_buffer buf;
    int ret;

    //unsigned int flength = 0;
    unsigned int offset = 0;
    unsigned int copy_count = 0;
    unsigned int remain = 0;
    unsigned int packet_id = 0;
    unsigned int frame_id = 0;

    struct timeval timestamp;
    size_t buffer_size = 0;
    ArvGvspPacket *packet;

    int stream_count = 0;

    //std::vector<ArvGvspPacket *> packet_buffers;

    tv.tv_sec = 0;
    tv.tv_usec = 300000;

    loop = 1;
    BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Enter.. " ;

    while(loop)
    {
        if(video_stream_mode == 1){
            stream_count = 1; 
        }
        else if(video_stream_mode == 2){
            stream_count = -1; 
        }
        else if(video_stream_mode == 0){
            stream_count = 0; 
        }

        FD_ZERO(&fds);
        FD_SET(m_video_fd, &fds);

        ret = select(m_video_fd+1, &fds, NULL, NULL, &tv);
        if (-1 == ret) {
            fprintf(stderr, "select error\n");
            //return 0;
        } 
        else if (0 == ret) {
            //fprintf(stderr, "timeout waiting for frame\n");
            usleep(30*1000);
            continue;
        }

        if (FD_ISSET(m_video_fd, &fds)) 
        {
          memset(&buf, 0, sizeof(buf));
          buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory = V4L2_MEMORY_MMAP;

          if (-1 == ioctl(m_video_fd, VIDIOC_DQBUF, &buf)) {
            fprintf(stderr, "VIDIOC_DQBUF failure\n");
            //return 1;
          }

          //flength = buf.bytesused;
          timestamp = buf.timestamp;

          //if(frame_id % 10 == 0){
          //    BOOST_LOG_TRIVIAL(info) << "frame length[" << buf.bytesused << "]";
          //    BOOST_LOG_TRIVIAL(info) << "frame timestamp[" << timestamp.tv_sec << ", " << timestamp.tv_usec << "]" ;
          //}

          offset = 0;
          copy_count = buf.bytesused / m_transfer_packet_length;
          remain = buf.bytesused % m_transfer_packet_length;

          if(bufferof_packets.size() == 0) {
              for(unsigned int i = 0; i < copy_count+8; i++){
                  bufferof_packets.push_back(new char[1536]);
              }
              BOOST_LOG_TRIVIAL(info) << copy_count << " packet buffer allocated" << std::endl; 
          }

          char *packet_source = (char *)v4l2_ubuffers[buf.index].start;
 
          packet_id = 0;
          buffer_size = 1536;

          /*  Leader packet   */ 
          packet = gvsp_packet_new (ARV_GVSP_CONTENT_TYPE_LEADER,
                            frame_id, packet_id, sizeof (ArvGvspImageLeader), bufferof_packets[packet_id], &buffer_size);
          packet_id++;
          if (packet != NULL) {
              ArvGvspImageLeader *leader;
  
              leader = (ArvGvspImageLeader *)gvsp_packet_get_data (packet);
              leader->flags = 0;
              leader->payload_type = htons (ARV_BUFFER_PAYLOAD_TYPE_IMAGE);
              //leader->timestamp_high = htonl (((uint64_t) timestamp >> 32));
              //leader->timestamp_low  = htonl ((uint64_t) timestamp & 0xffffffff);
              leader->infos.pixel_format = htonl (0x020C0112);
              leader->infos.width = htonl (IMAGE_WIDTH);
              leader->infos.height = htonl (IMAGE_HEIGHT);
              leader->infos.x_offset = htonl (0);
              leader->infos.y_offset = htonl (0);
              leader->infos.x_padding = htonl (0);
              leader->infos.y_padding = htonl (0);

              if(stream_count > 0){
                  frame_udp_send ((char *)packet, buffer_size); 
              }
              else if(stream_count < 0){
                  frame_udp_send ((char *)packet, buffer_size); 
              }
          }

          buffer_size = 1536;
          while(copy_count > 0) {
              packet = gvsp_packet_new (ARV_GVSP_CONTENT_TYPE_PAYLOAD,
                             frame_id, packet_id, m_transfer_packet_length, bufferof_packets[packet_id], &buffer_size);
              packet_id++;
              if (packet != NULL){
                  memcpy (gvsp_packet_get_data (packet), (char *)(packet_source+offset), m_transfer_packet_length);

                  if(stream_count > 0){
                      frame_udp_send ((char *)packet, buffer_size);
                  }
                  else if(stream_count < 0){
                      frame_udp_send ((char *)packet, buffer_size);
                  }
              }
              else{
                  std::cout << "payload packet is NULL " << std::endl;
              }

              copy_count--;
              offset += m_transfer_packet_length;
          }
          
          buffer_size = 1536;
          if(remain != 0){
              packet = gvsp_packet_new (ARV_GVSP_CONTENT_TYPE_PAYLOAD,
                             frame_id, packet_id, remain, bufferof_packets[packet_id], &buffer_size);
              packet_id++;
              if (packet != NULL) {
                  memcpy (gvsp_packet_get_data (packet),  packet_source+(offset), remain);

                  if(stream_count > 0){
                      frame_udp_send ((char *)packet, buffer_size); 
                  }
                  else if(stream_count < 0){
                      frame_udp_send ((char *)packet, buffer_size); 
                  }
              }
          }

          buffer_size = 1536;
          /*  Trailer packet   */ 
          packet = gvsp_packet_new (ARV_GVSP_CONTENT_TYPE_TRAILER,
                            frame_id, packet_id, sizeof (ArvGvspTrailer), bufferof_packets[packet_id], &buffer_size);
          packet_id++;
          if (packet != NULL) {
              ArvGvspTrailer *trailer;
  
              trailer = (ArvGvspTrailer *)gvsp_packet_get_data (packet);
              trailer->payload_type = htonl (ARV_BUFFER_PAYLOAD_TYPE_IMAGE);
              trailer->data0 = 0;

              if(stream_count > 0){
                  stream_count--;
                  frame_udp_send ((char *)packet, buffer_size); 
              }
              else if(stream_count < 0){
                  frame_udp_send ((char *)packet, buffer_size); 
              }

              //std::cout << "Send TRAILER (stream_count: " << stream_count << ")" << std::endl;
              if(stream_count == 0){
                  video_stream_mode = 0;
              }
          }

          if(frame_id % 100 == 0){
              BOOST_LOG_TRIVIAL(info) << "packets of frame complete [" << buf.bytesused << "]";
          }

          frame_id++;

          buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory = V4L2_MEMORY_MMAP;
          if (-1 == ioctl(m_video_fd, VIDIOC_QBUF, &buf)) {
            fprintf(stderr, "VIDIOC_QBUF failure\n");
          }

          //BOOST_LOG_TRIVIAL(info) << "Queue buffer" << buf.index << std::endl; 
        }
        
    }

    if (v4l2_streamoff(m_video_fd) == -1) {
      BOOST_LOG_TRIVIAL(error) << "Error Stream OFF ..." << std::endl; 
    }

    if(bufferof_packets.size() != 0) {
      std::vector<char *>::iterator iter;
      for(iter = bufferof_packets.begin(); iter != bufferof_packets.end(); iter++){
          //char *ptr = *iter;
          //delete[] ptr;
          delete[] *iter;
      }
      bufferof_packets.clear();
      BOOST_LOG_TRIVIAL(info) << "packet buffer freed" << std::endl; 
    }

  }
#endif

#if 0
  void videoframe_polling()
  {
    //struct timeval tv = {.tv_sec = 3, .tv_usec = 0};
    //int ret;
    unsigned int flength = 0;
    unsigned int offset = 0;
    unsigned int copy_count = 0;
    unsigned int remain = 0;
    unsigned int packet_id = 0;
    unsigned int frame_id = 0;

    //struct timeval timestamp;
    size_t buffer_size;
    ArvGvspPacket *packet;

    char frame_data[16*1024];
    int stream_count = 0;

    struct stat f_info;
    char *f_map;

    //std::vector<ArvGvspPacket *> packet_buffers;
    //tv.tv_sec = 0;
    //tv.tv_usec = 300000;

    m_fd = -1;
    //m_fd = open("./test_1.yuv", O_RDONLY, (mode_t)0600);
    m_fd = open("./test_1_y.yuv", O_RDONLY, (mode_t)0600);
    if(m_fd  < 0){
      BOOST_LOG_TRIVIAL(error) << "Sample file Open error ...";
      flength = 16*1024;
    }

    fstat(m_fd, &f_info);
    BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": sample size size= " << f_info.st_size ;

    flength = f_info.st_size;
    f_map = (char *)mmap(0, f_info.st_size, PROT_READ, MAP_SHARED, m_fd, 0);
    if(f_map == NULL){
      BOOST_LOG_TRIVIAL(error) << "Mmap error ...";
      flength = 0;
    }

    loop = 1;

    while(loop)
    {
        if(video_stream_mode == 1){
            stream_count = 1; 
        }
        else if(video_stream_mode == 2){
            stream_count = -1; 
        }
        else if(video_stream_mode == 0){
            stream_count = 0; 
        }

        //timestamp = 0;
        if(frame_id % 10 == 0){
            BOOST_LOG_TRIVIAL(info) << "frame length[" << flength << "]";
            //BOOST_LOG_TRIVIAL(info) << "frame timestamp[" << timestamp.tv_sec << ", " << timestamp.tv_usec << "]" ;
        }

        offset = 0;
        copy_count = flength / 1480;
        remain = flength % 1480;

        if(bufferof_packets.size() == 0) {
            for(unsigned int i = 0; i < copy_count+8; i++){
                bufferof_packets.push_back(new char[1536]);
            }
            BOOST_LOG_TRIVIAL(info) << copy_count << " packet buffer allocated" << std::endl; 
        }

        //char *packet_source = frame_data;
        char *packet_source = f_map;
        packet_id = 0;

        buffer_size = 1536;
#if 1
          /*  Leader packet   */ 
          packet = gvsp_packet_new (ARV_GVSP_CONTENT_TYPE_LEADER,
                            frame_id, packet_id, sizeof (ArvGvspImageLeader), bufferof_packets[packet_id], &buffer_size);
          packet_id++;
          if (packet != NULL) {
              ArvGvspImageLeader *leader;
  
              leader = (ArvGvspImageLeader *)gvsp_packet_get_data (packet);
              leader->flags = 0;
              leader->payload_type = htons (ARV_BUFFER_PAYLOAD_TYPE_IMAGE);
              //leader->timestamp_high = htonl (((uint64_t) timestamp >> 32));
              //leader->timestamp_low  = htonl ((uint64_t) timestamp & 0xffffffff);
              leader->infos.pixel_format = htonl (0x020C0112);
              leader->infos.width = htonl (IMAGE_WIDTH);
              leader->infos.height = htonl (IMAGE_HEIGHT);
              leader->infos.x_offset = htonl (0);
              leader->infos.y_offset = htonl (0);
              leader->infos.x_padding = htonl (0);
              leader->infos.y_padding = htonl (0);

              if(stream_count > 0){
                  //std::cout << "frame_udp_send count, stream_count=" << stream_count << std::endl;
                  frame_udp_send ((char *)packet, buffer_size); 
              }
              else if(stream_count < 0){
                  //std::cout << "frame_udp_send continue" << std::endl;
                  frame_udp_send ((char *)packet, buffer_size); 
              }
          }
          else{
              std::cout << "packet is NULL" << std::endl;
          }

          buffer_size = 1536;
          while(copy_count > 0) {
              packet = gvsp_packet_new (ARV_GVSP_CONTENT_TYPE_PAYLOAD,
                             frame_id, packet_id, 1480, bufferof_packets[packet_id], &buffer_size);
              packet_id++;
              if (packet != NULL){
                  //memcpy (gvsp_packet_get_data (packet), data, size);
                  //std::cout << "buffer_size: " << buffer_size << std::endl;
                  memcpy (gvsp_packet_get_data (packet), (char *)(packet_source+offset), 1480);

                  if(stream_count > 0){
                      frame_udp_send ((char *)packet, buffer_size);
                  }
                  else if(stream_count < 0){
                      frame_udp_send ((char *)packet, buffer_size);
                  }
              }
              else{
                  std::cout << "payload packet is NULL " << std::endl;
              }

              copy_count--;
              offset += 1480;
          }
          
          if(remain != 0) {
              packet = gvsp_packet_new (ARV_GVSP_CONTENT_TYPE_PAYLOAD,
                             frame_id, packet_id, remain, bufferof_packets[packet_id], &buffer_size);
              packet_id++;
              if (packet != NULL) {
                  memcpy (gvsp_packet_get_data (packet),  packet_source+(offset), remain);
                  
                  if(stream_count > 0){
                      frame_udp_send ((char *)packet, buffer_size); 
                  }
                  else if(stream_count < 0){
                      frame_udp_send ((char *)packet, buffer_size); 
                  }
              }
          }

          /*  Trailer packet   */ 
          packet = gvsp_packet_new (ARV_GVSP_CONTENT_TYPE_TRAILER,
                            frame_id, packet_id, sizeof (ArvGvspTrailer), bufferof_packets[packet_id], &buffer_size);
          packet_id++;
          if (packet != NULL) {
              ArvGvspTrailer *trailer;
  
              trailer = (ArvGvspTrailer *)gvsp_packet_get_data (packet);
              trailer->payload_type = htonl (ARV_BUFFER_PAYLOAD_TYPE_IMAGE);
              trailer->data0 = 0;

              //frame_udp_send ((char *)packet, buffer_size); 
              if(stream_count > 0){
                  stream_count--;
                  frame_udp_send ((char *)packet, buffer_size); 
              }
              else if(stream_count < 0){
                  frame_udp_send ((char *)packet, buffer_size); 
              }

              //std::cout << "Send TRAILER (stream_count: " << stream_count << ")" << std::endl;
              if(stream_count == 0){
                  video_stream_mode = 0;
              }
          }
#endif
          if(frame_id % 10 == 0){
              BOOST_LOG_TRIVIAL(info) << "packets of frame complete [" << flength << "]";
          }

          //frames.push(packet_buffers); 
          frame_id++;

        //loop--;
        usleep(500*1000);
     }

     if(bufferof_packets.size() != 0) {
         std::vector<char *>::iterator iter;
         for(iter = bufferof_packets.begin(); iter != bufferof_packets.end(); iter++){
             //char *ptr = *iter;
             //delete[] ptr;
             delete[] *iter;
         }
         bufferof_packets.clear();
         BOOST_LOG_TRIVIAL(info) << "packet buffer freed" << std::endl; 
     }

     BOOST_LOG_TRIVIAL(error) << "f_map Unmmap ...";
     munmap(f_map, f_info.st_size);

     BOOST_LOG_TRIVIAL(error) << "m_fd Close ...";
     close(m_fd);
  }
#endif

  int startDevice()
  {
    int ret = -1;

    BOOST_LOG_TRIVIAL(info) << "Stream ON ..." << std::endl; 
    if (v4l2_streamon(m_video_fd) == -1) {
      BOOST_LOG_TRIVIAL(error) << "Error Stream ON ..." << std::endl; 
      return ret;
    }

    ret = 1;
    std::function<void()> func = std::bind(&UDPAsyncServer::videoframe_polling, this);
    frame_proc = std::thread(func);

    return ret;
  }

  int stopDevice()
  {
    int ret = -1;
#if 0
    if (v4l2_streamoff(m_video_fd) == -1) {
      BOOST_LOG_TRIVIAL(error) << "Error Stream OFF ..." << std::endl; 
      return ret;
    }
#endif
    ret = 1;

    loop = 0;
    if(frame_proc.joinable()){
        BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Join Wait ..." ;
        frame_proc.join();
    }

    return ret;
  }

  void signal_handler(int signal)
  {
      gSignalStatus = signal;
      socket.cancel();
  }

  int video_stream_mode ;

private:
  int m_video_fd = -1;
  //unsigned int m_frame_id = 0;
  //unsigned int m_packet_id = 0;

  asio::ip::udp::socket socket;
  asio::ip::udp::endpoint remote_peer;
  char buffer[MAXBUF];

  volatile std::sig_atomic_t gSignalStatus;
  boost::atomic<bool> init_done;

  std::atomic<int> loop ;
  std::thread frame_proc ;

  std::queue< std::vector<ArvGvspPacket *> > frames;
  std::vector<char *> bufferof_packets;

  unsigned int m_transfer_packet_length;
  int m_fd;
};


class UDPAsyncCMDServer 
{

public:
  UDPAsyncCMDServer(asio::io_service& service, unsigned short port) 
     : socket(service, 
          asio::ip::udp::endpoint(asio::ip::udp::v4(), port))
  {  
    cmd_count = 0;
    waitForReceive();  
  }

  ~UDPAsyncCMDServer() 
  { 
      //std::cout << __FUNCTION__ << ": Exec.. " << std::endl;
#if BOOST_LOG_DYN_LINK == 1
      BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Exec..." << std::endl; 
#endif
      socket.cancel();
  }


  ArvGvcpPacketType arv_gvcp_packet_get_packet_type (ArvGvcpPacket *packet)
  {   
    if (packet == NULL)
        return ARV_GVCP_PACKET_TYPE_ERROR;

    return (ArvGvcpPacketType) packet->header.packet_type;
  }

  ArvGvcpCommand arv_gvcp_packet_get_command (ArvGvcpPacket *packet)
  {
    if (packet == NULL)
        return (ArvGvcpCommand) 0;

    return (ArvGvcpCommand) ntohs (packet->header.command);
  }

  uint16_t arv_gvcp_packet_get_packet_id (ArvGvcpPacket *packet)
  {
    if (packet == NULL)
        return 0;

    return ntohs (packet->header.id);
  }

  void arv_gvcp_packet_get_read_memory_cmd_infos (const ArvGvcpPacket *packet, uint32_t *address, uint32_t *size)
  {           
    if (packet == NULL) 
    {
        if (address != NULL) 
            *address = 0;

        if (size != NULL)
            *size = 0;

        return;
    }

    if (address != NULL)
        *address = ntohl (*((uint32_t *) ((char *) packet + sizeof (ArvGvcpPacket))));

    if (size != NULL)
        *size = (ntohl (*((uint32_t *) ((char *) packet + sizeof (ArvGvcpPacket) + sizeof (uint32_t))))) & 0xffff;
  }


  ArvGvcpPacket *
  arv_gvcp_packet_new_discovery_ack (uint16_t packet_id, size_t *packet_size)
  {   
    ArvGvcpPacket *packet;
    
    //g_return_val_if_fail (packet_size != NULL, NULL);
        
    *packet_size = sizeof (ArvGvcpHeader) + ARV_GVBS_DISCOVERY_DATA_SIZE ;

    packet = (ArvGvcpPacket *)malloc (*packet_size);
        
    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_ACK; 
    packet->header.packet_flags = 0;
    packet->header.command = htons (ARV_GVCP_COMMAND_DISCOVERY_ACK); 
    packet->header.size = htons (ARV_GVBS_DISCOVERY_DATA_SIZE);
    packet->header.id = htons (packet_id);

    return packet;
  }    

  size_t arv_gvcp_packet_get_read_register_ack_size (void)
  {
    return sizeof (ArvGvcpHeader) + sizeof (uint32_t); 
  }

  void arv_gvcp_packet_get_write_register_cmd_infos (const ArvGvcpPacket *packet, uint32_t *address, uint32_t *value)
  {
    if (packet == NULL) {
        if (address != NULL)
            *address = 0;

        if (value != NULL)
            *value = 0;

        return;
    }

    if (address != NULL)
        *address = ntohl (*((uint32_t *) ((char *) packet + sizeof (ArvGvcpPacket))));

    if (value != NULL)
        *value = ntohl (*((uint32_t *) ((char *) packet + sizeof (ArvGvcpPacket) + sizeof (uint32_t))));
  }


  void arv_gvcp_packet_get_read_register_cmd_infos (const ArvGvcpPacket *packet, uint32_t *address)
  {
    if (packet == NULL) {
        if (address != NULL)
            *address = 0;
        return;
    }

    if (address != NULL) 
        *address = ntohl (*((uint32_t *) ((char *) packet + sizeof (ArvGvcpPacket))));
  }

  uint32_t arv_gvcp_packet_get_read_register_ack_value (const ArvGvcpPacket *packet)
  {
    if (packet == NULL)
        return 0;
    return ntohl (*((uint32_t *) ((char *) packet + sizeof (ArvGvcpPacket))));
  }

  bool gev_camera_read_memory (uint32_t address, uint32_t size, void *buffer)
  {
    uint32_t read_size;

    //g_return_val_if_fail (ARV_IS_FAKE_CAMERA (camera), FALSE);
    //g_return_val_if_fail (buffer != NULL, FALSE);
    //g_return_val_if_fail (size > 0, FALSE);

    if (address < ARV_FAKE_CAMERA_MEMORY_SIZE) {
        read_size = std::min(address + size, (uint32_t)ARV_FAKE_CAMERA_MEMORY_SIZE) - address;
 
        std::cout << "[" << __FUNCTION__ << "] read_size: " << read_size << std::endl;
        memcpy (buffer, (char *)(g_camera_memory+address), read_size);

        if (read_size == size){
            std::cout << "[" << __FUNCTION__ << "] return true => " << read_size << std::endl;
            return true;
        }

        //size = size - read_size;
        //address = ARV_FAKE_CAMERA_MEMORY_SIZE;
        //buffer = ((char *) buffer) + read_size;
    }

    //address -= ARV_FAKE_CAMERA_MEMORY_SIZE;
    //read_size = CMIN (address + size, camera->priv->genicam_xml_size) - address;

    //memcpy (buffer, ((char *) camera->priv->genicam_xml) + address, read_size);
    //if (read_size < size)
    //    memset (((char *) buffer) + read_size, 0, size - read_size);

    return true;
  }

  void waitForReceive() 
  {
      //int ret = 0;
      //std::cout<<"Listening at : " << socket.local_endpoint() << std::endl;
#if 0
      socket.async_receive_from( 
           asio::buffer(buffer, MAXBUF), 
           remote_peer,
           boost::bind(&UDPAsyncCMDServer::DataReceive, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred) );
#else
      socket.async_receive_from( 
           asio::buffer(buffer, MAXBUF), 
           remote_peer,
           boost::bind(&UDPAsyncCMDServer::DataReceive, this, _1, _2) );
#endif

      //socket.async_receive_from( 
      //     asio::buffer(buffer, MAXBUF), 
      //     remote_peer,
      //     boost::bind(&UDPAsyncCMDServer::DataReceive, this,
      //          boost::asio::placeholders::error,
      //          boost::asio::placeholders::bytes_transferred, cmd_count++) );

  }

  void send_complete (const sys::error_code& ec, size_t sz) 
  {
#if BOOST_LOG_DYN_LINK == 1
      BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Exec..." << std::endl; 
#endif
  }

  void DataReceive (const sys::error_code& ec, size_t sz, int num) 
  {
      //ArvGvcpPacket *gvcp_packet = (ArvGvcpPacket *)buffer;

      if( ec == boost::asio::error::operation_aborted ) 
      {
          std::cout << __FUNCTION__ << ": Aborted "  << '\n';
          return;
      }

      waitForReceive();

      std::cout << "received size: " << sz << std::endl;
      std::cout << "received count: " << num << std::endl;
      std::cout << "Received from " << remote_peer << '\n';

  }

  void DataReceive (const sys::error_code& ec, size_t sz) 
  {
      //const char *msg = "hello from server";
      ArvGvcpPacket *gvcp_packet = (ArvGvcpPacket *)buffer;

      std::cout << "error_code: " << ec << '\n';
      std::cout << "messge_size: " << boost::asio::error::message_size << std::endl;
      std::cout << "received size: " << sz << std::endl;
      
      if( ec == boost::asio::error::operation_aborted ) 
      {
          std::cout << __FUNCTION__ << ": Aborted "  << '\n';
          return;
      }

      //std::cout << "Received: [" << buffer << "] " << remote_peer << '\n';
      std::cout << "Received from " << remote_peer << '\n';
      switch (ntohs (gvcp_packet->header.command)) 
      {
        uint32_t register_address;
        uint32_t register_value;
        uint32_t be_value ;
        bool success;

        case ARV_GVCP_COMMAND_DISCOVERY_CMD:
            BOOST_LOG_TRIVIAL(info) <<"["<<__FUNCTION__<<" "<< __LINE__<<"] "<<"GVCP_Discovery Command ";

            //ack_packet = arv_gvcp_packet_new_discovery_ack (packet_id, &ack_packet_size);
            //gev_camera_read_memory (0, ARV_GVBS_DISCOVERY_DATA_SIZE, &ack_packet->data);

            break;

        case ARV_GVCP_COMMAND_READ_MEMORY_CMD:
            BOOST_LOG_TRIVIAL(info) <<"["<<__FUNCTION__<<"] "<<"GVCP_ReadMemory Command ";
            break;

        case ARV_GVCP_COMMAND_WRITE_MEMORY_CMD:
            BOOST_LOG_TRIVIAL(info) <<"["<<__FUNCTION__<<"] "<<"GVCP_WriteMemory Command ";
            break;

        case ARV_GVCP_COMMAND_READ_REGISTER_CMD:

            BOOST_LOG_TRIVIAL(info) <<"["<<__FUNCTION__<<"] "<<"GVCP_ReadRegister Command ";

            arv_gvcp_packet_get_read_register_cmd_infos (gvcp_packet, &register_address);
            BOOST_LOG_TRIVIAL(info) <<"["<<__FUNCTION__<<"] "<< "Read register command 0x" << std::hex << register_address ;

            success = gev_camera_read_memory (register_address, sizeof (register_value), &be_value);
            //          register_address, register_value);
            //ack_packet = arv_gvcp_packet_new_read_register_ack (register_value, packet_id,
            //                            &ack_packet_size);
            ((void)success);
            break;

        case ARV_GVCP_COMMAND_WRITE_REGISTER_CMD:
            //uint32_t register_address;
            //uint32_t register_value;

            register_value = 0;
            BOOST_LOG_TRIVIAL(info) <<"["<<__FUNCTION__<<"] "<<"GVCP_WriteRegister Command ";
            arv_gvcp_packet_get_write_register_cmd_infos (gvcp_packet, &register_address, &register_value);

            BOOST_LOG_TRIVIAL(info) <<"["<<__FUNCTION__<<"] "<< "Read register address = 0x" << std::hex << register_address ;
            BOOST_LOG_TRIVIAL(info) <<"["<<__FUNCTION__<<"] "<< "Read register value = " << std::hex << register_value ;

#if 1
            if(register_value == 1){
                m_stream_server->video_stream_mode = 1;
                //m_stream_server->startDevice();
            }
            else if(register_value == 2){
                m_stream_server->video_stream_mode = 2;
                //m_stream_server->startDevice();
            }
            else if(register_value == 0){
                m_stream_server->video_stream_mode = 0;
                //m_stream_server->stopDevice();
            }
#endif
            break;

       default:
            BOOST_LOG_TRIVIAL(error) <<"["<<__FUNCTION__<<"] "<<"Unknown command";

      }

      waitForReceive();
#if 0
      socket.async_send_to(
          asio::buffer(msg, strlen(msg)),
          remote_peer,
          boost::bind(&UDPAsyncCMDServer::send_complete, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred) );
#endif
  }

  void signal_handler(int signal)
  {
      gSignalStatus = signal;
      socket.cancel();
  }

  UDPAsyncServer *m_stream_server;

private:
  asio::ip::udp::socket socket;
  asio::ip::udp::endpoint remote_peer;
  char buffer[MAXBUF];

  volatile std::sig_atomic_t gSignalStatus;
  int cmd_count;

};

//asio::io_service service;
//asio::io_service command_service;

UDPAsyncServer *server = nullptr;
UDPAsyncCMDServer *command_server = nullptr;

void signal_handler(int signal)
{
    std::cout << "Signal exec.. " << '\n';
#if 0
    if(server != nullptr)
        server->signal_handler(signal);

    if(command_server != nullptr)
        command_server->signal_handler(signal);
#endif
    if(server != nullptr)
        delete server;

    if(command_server != nullptr)
        delete command_server;
}

void stream_proc(void *pdat)
{
    asio::io_service *srv = (asio::io_service *)pdat;
    srv->run();
}

void command_proc(void *pdat)
{
    asio::io_service *srv = (asio::io_service *)pdat;
    srv->run();
}

int main() 
{
  asio::io_service service;
  asio::io_service command_service;

  server = new UDPAsyncServer(service, 55000);

  command_server = new UDPAsyncCMDServer(command_service, ARV_GVCP_PORT);
  command_server->m_stream_server = server;

  server->initDevice("/dev/video0");

  //std::signal(SIGINT, server.signal_handler);
  std::signal(SIGINT, signal_handler);

  //service.run();
  std::function<void()> func = std::bind(&stream_proc, &service);
  std::thread my_thread(func);

  std::function<void()> func1 = std::bind(&command_proc, &command_service);
  std::thread my_thread1(func1);

  server->video_stream_mode = 0;
  server->startDevice();

  std::cout << "service run.. " << '\n';

  my_thread.join();
  my_thread1.join();
}


