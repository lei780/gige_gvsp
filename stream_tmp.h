
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

namespace asio = boost::asio;
namespace sys = boost::system;
using namespace boost::placeholders;

const size_t MAXBUF = 256;
char g_camera_memory[1024];

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

  virtual ~UDPAsyncServer() ;

  void waitForReceive(void);
  void send_complete (const sys::error_code& ec, size_t sz);
  void frame_udp_send (char *pdata, unsigned int length) ;
  void DataReceive (const sys::error_code& ec, size_t sz) ;

  bool packet_has_extended_ids (const PvcGvspPacket *packet);
  void * gvsp_packet_get_data (const PvcGvspPacket *packet);

  PvcGvspPacket * gvsp_packet_new (ArvGvspContentType content_type,
             uint16_t frame_id, uint32_t packet_id, size_t data_size, void *buffer, size_t *buffer_size);

  int initDevice(const char *dev_name);
  void videoframe_polling(void);

  int PvcStartDevice(void);
  int PvcStopDevice(void);

  //void signal_handler(int signal);

  int video_stream_mode ;

private:
  int m_video_fd = -1;
  //unsigned int m_frame_id = 0;
  //unsigned int m_packet_id = 0;

  asio::ip::udp::socket socket;
  asio::ip::udp::endpoint remote_peer;
  char buffer[MAXBUF];

  //volatile std::sig_atomic_t gSignalStatus;

  boost::atomic<bool> init_done;
  boost::atomic<int> loop ;

  std::thread frame_proc ;
  std::mutex m_mode_lock;

  std::queue< std::vector<ArvGvspPacket *> > frames;
  std::vector<char *> bufferof_packets;

  unsigned int m_transfer_packet_length;
  int m_fd;
}



