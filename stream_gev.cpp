/******************************************************************************
*
* Copyright (C) 2005 - 2024 Powerlogics, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Powerlogics device, or
* (b) that interact with a Powerlogics device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* POWERLOGICS  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Powerlogics shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Powerlogics.
*
******************************************************************************/

#include <iostream>
#include <thread>
#include <queue>
#include <vector>
#include <list>
#include <functional>
#include <mutex>

#include <csignal>
#include <cstdio>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/bind/bind.hpp>
#include <boost/atomic.hpp>

#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/videodev2.h>

#include "v4l2_driver.h"
#include "gvcp_header.h"

#include "stream_gev.h"

using namespace std; 

namespace asio = boost::asio;
namespace sys = boost::system;
using namespace boost::placeholders;

//const size_t MAXBUF = 256;
//char g_camera_memory[1024];
//std::queue< std::vector<ArvGvspPacket *> > frames;

UDPStreamServer::~UDPStreamServer() 
{ 
    PvcStopDevice();

    //std::cout << __FUNCTION__ << ": Exec.. " << std::endl;
    std::cout << __FUNCTION__ << ": Enter ..." << std::endl; 

  if(init_done){
    if (v4l2_munmap() == -1) {
      std::cout << __FUNCTION__ << ": Error Unmap ..." << std::endl; 
    }

    if (v4l2_close(m_video_fd) == -1) {
      std::cout << __FUNCTION__ << ": Error Close ..." << std::endl; 
    }
  }

  loop = 0;
  if(frame_proc.joinable()){
      std::cout << __FUNCTION__ << ": Join Wait ..." ;
      frame_proc.join();
  }

  socket.cancel();
}

void UDPStreamServer::waitForReceive() 
{
    socket.async_receive_from( 
         asio::buffer(buffer, MAX_FRAME_BUFFER), 
         remote_peer,
         boost::bind(&UDPStreamServer::DataReceive, this, _1, _2) );
}

void UDPStreamServer::send_complete (const sys::error_code& ec, size_t sz) 
{
    std::cout << __FUNCTION__ << ": Exec..." ;
}

void UDPStreamServer::frame_udp_send (char *pdata, unsigned int length) 
{
    //std::cout << "Send to " << remote_peer << '\n';

    //socket.async_send_to(
    //    asio::buffer(pdata, length),
    //    remote_peer,
    //    boost::bind(&UDPAsyncServer::send_complete, this,
    //          boost::asio::placeholders::error,
    //          boost::asio::placeholders::bytes_transferred) );

    socket.send_to(asio::buffer(pdata, length), remote_peer);  
}

void UDPStreamServer::DataReceive (const sys::error_code& ec, size_t sz) 
{
    if( ec == boost::asio::error::operation_aborted ) 
    {
        std::cout << __FUNCTION__ << ": Aborted "  << '\n';
        return;
    }
    //std::cout << "Received: [" << buffer << "] " << remote_peer << '\n';
    waitForReceive() ;
}


bool UDPStreamServer::gvsp_packet_has_extended_ids (const PvcGvspPacket *packet)
{
  return (packet->header[2] & ARV_GVSP_PACKET_EXTENDED_ID_MODE_MASK) != 0;
}


void * UDPStreamServer::gvsp_packet_get_data (const PvcGvspPacket *packet)
{
  if (gvsp_packet_has_extended_ids (packet)) {
      PvcGvspExtendedHeader *header = (PvcGvspExtendedHeader *) &packet->header;

      return &header->data;
  } else {
      PvcGvspHeader *header = (PvcGvspHeader *) &packet->header;

      return &header->data;
  }
}


PvcGvspPacket * UDPStreamServer::gvsp_packet_new (PvcGvspContentType content_type,
           uint16_t frame_id, uint32_t packet_id, size_t data_size, void *buffer, size_t *buffer_size)
{
   PvcGvspPacket *packet;
   PvcGvspHeader *header;
   size_t packet_size;

   packet_size = sizeof (PvcGvspPacket) + sizeof (PvcGvspHeader) + data_size;
   if (packet_size == 0 || (buffer != NULL && (buffer_size == NULL || packet_size > *buffer_size)))
       return NULL;

   if (buffer_size != NULL)
       *buffer_size = packet_size;

   if (buffer != NULL){
       packet = (PvcGvspPacket *)buffer;
   }
   else{
       packet = (PvcGvspPacket *)malloc(packet_size);
       std::cout << __FUNCTION__ << ": alloc " << std::endl;
   }

   packet->packet_type = 0;

   //header = (void *) &packet->header;
   header = (PvcGvspHeader *)packet->header;
   header->frame_id = htons (frame_id);
   header->packet_infos = htonl ((packet_id & ARV_GVSP_PACKET_ID_MASK) |
                   ((content_type << ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_POS) &
                    ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_MASK));

   return packet;
}


int UDPStreamServer::PvcInitDevice(const char *dev_name)
{
  m_video_fd = v4l2_open(dev_name);
  if (m_video_fd == -1) {
    //fprintf(stderr, "can't open %s\n", dev_name);
    std::cout << dev_name << ": Can't Open ..\n" ;
    return m_video_fd;
  }

  if (v4l2_querycap(m_video_fd, dev_name) == -1) {
    std::cout << ": Query ...\n" ;
    return -1;
  }

  // most of devices support YUYV422 packed.
  //if (v4l2_sfmt(m_video_fd, V4L2_PIX_FMT_MJPEG) == -1)
  //if (v4l2_sfmt(m_video_fd, V4L2_PIX_FMT_YUYV) == -1)
  if (v4l2_sfmt(m_video_fd, V4L2_PIX_FMT_NV12) == -1)
  {
    std::cout << "Error Set Format ..." << std::endl; 
    return -1;
  }

  //if (v4l2_gfmt(video_fildes) == -1) {
  //  perror("v4l2_gfmt");
  //  return 1;
  //}

  if (v4l2_sfps(m_video_fd, 20) == -1) { // no fatal error
    //perror("v4l2_sfps");
    std::cout << "Error Set Frame rate ..." << std::endl; 
    return -1;
  }

  if (v4l2_mmap(m_video_fd) == -1) {
    //perror("v4l2_mmap");
    std::cout << "Error Set MMAP ..." << std::endl; 
    return -1;
  }

  init_done = true;
  return m_video_fd;
}


void UDPStreamServer::PvcVideoframePolling()
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
  PvcGvspPacket *packet;

  int stream_count = 0;

  //std::vector<ArvGvspPacket *> packet_buffers;

  tv.tv_sec = 0;
  tv.tv_usec = 300000;

  loop = 1;
  std::cout << __FUNCTION__ << ": Enter.. " << std::endl;

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
            std::cout << copy_count << " packet buffer allocated" << std::endl; 
        }

        char *packet_source = (char *)v4l2_ubuffers[buf.index].start;

        packet_id = 0;
        buffer_size = 1536;

        /*  Leader packet   */ 
        packet = gvsp_packet_new (ARV_GVSP_CONTENT_TYPE_LEADER,
                          frame_id, packet_id, sizeof (PvcGvspImageLeader), bufferof_packets[packet_id], &buffer_size);
        packet_id++;
        if (packet != NULL) {
            PvcGvspImageLeader *leader;

            leader = (PvcGvspImageLeader *)gvsp_packet_get_data (packet);
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
                          frame_id, packet_id, sizeof (PvcGvspTrailer), bufferof_packets[packet_id], &buffer_size);
        packet_id++;
        if (packet != NULL) {
            PvcGvspTrailer *trailer;

            trailer = (PvcGvspTrailer *)gvsp_packet_get_data (packet);
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

        if(frame_id % 200 == 0){
            std::cout << "packets of frame complete [" << buf.bytesused << "]";
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
    std::cout << "Error Stream OFF ..." << std::endl; 
  }

  if(bufferof_packets.size() != 0) {
    std::vector<char *>::iterator iter;
    for(iter = bufferof_packets.begin(); iter != bufferof_packets.end(); iter++){
        //char *ptr = *iter;
        //delete[] ptr;
        delete[] *iter;
    }
    bufferof_packets.clear();
    std::cout << "packet buffer freed" << std::endl; 
  }

}


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
              std::cout << "packets of frame complete [" << flength << "]";
          }

          //frames.push(packet_buffers); 
          frame_id++;

        //loop--;
        //usleep(500*1000);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
     }

     if(bufferof_packets.size() != 0) {
         std::vector<char *>::iterator iter;
         for(iter = bufferof_packets.begin(); iter != bufferof_packets.end(); iter++){
             //char *ptr = *iter;
             //delete[] ptr;
             delete[] *iter;
         }
         bufferof_packets.clear();
         std::cout << "packet buffer freed" << std::endl; 
     }

     std::cout << "f_map Unmmap ...";
     munmap(f_map, f_info.st_size);

     std::cout << "m_fd Close ...";
     close(m_fd);
  }
#endif


int UDPStreamServer::PvcStartDevice()
{
  int ret = -1;

  std::cout << "Stream ON ..." << std::endl; 
  if (v4l2_streamon(m_video_fd) == -1) {
    std::cout << "Error Stream ON ..." << std::endl; 
    return ret;
  }

  ret = 1;
  std::function<void()> func = std::bind(&UDPStreamServer::PvcVideoframePolling, this);
  frame_proc = std::thread(func);

  return ret;
}


int UDPStreamServer::PvcStopDevice()
{
  int ret = -1;

  //if (v4l2_streamoff(m_video_fd) == -1) {
  //  BOOST_LOG_TRIVIAL(error) << "Error Stream OFF ..." << std::endl; 
  //  return ret;
  //}

  ret = 1;

  loop = 0;
  if(frame_proc.joinable()){
      std::cout << __FUNCTION__ << ": Join Wait ..." ;
      frame_proc.join();
  }

  return ret;
}



