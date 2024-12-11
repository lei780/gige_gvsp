
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

#include "gvcp_header.h"
#include "gev_gvcp.h"

namespace asio = boost::asio;
namespace sys = boost::system;
using namespace boost::placeholders;

const size_t MAXBUF = 256;
char g_camera_memory[1024];

class UDPAsyncCMDServer 
{
public:
  UDPAsyncCMDServer(boost::asio::io_service& service, unsigned short port) 
     : socket(service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port))
  {  
    cmd_count = 0;
    waitForReceive();  
  }

  virtual ~UDPAsyncCMDServer() ;

  PvcGvcpPacketType gvcp_packet_get_packet_type (PvcGvcpPacket *packet);
  PvcGvcpCommand gvcp_packet_get_command (PvcGvcpPacket *packet);

  PvcGvcpPacket * gvcp_packet_new_discovery_ack (uint16_t packet_id, size_t *packet_size);

  size_t gvcp_packet_get_read_register_ack_size (void);
  void gvcp_packet_get_read_memory_cmd_infos (const PvcGvcpPacket *packet, uint32_t *address, uint32_t *size);
  uint16_t gvcp_packet_get_packet_id (ArvGvcpPacket *packet);
  void gvcp_packet_get_write_register_cmd_infos (const ArvGvcpPacket *packet, uint32_t *address, uint32_t *value);
  void gvcp_packet_get_read_register_cmd_infos (const ArvGvcpPacket *packet, uint32_t *address);
  uint32_t arv_gvcp_packet_get_read_register_ack_value (const ArvGvcpPacket *packet);
  bool gev_camera_read_memory (uint32_t address, uint32_t size, void *buffer);

  PvcGvcpPacket * gvcp_packet_new_read_register_ack (uint32_t value, uint16_t packet_id, size_t *packet_size);
  PvcGvcpPacket * gvcp_packet_new_read_register_cmd (uint32_t address, uint16_t packet_id, size_t *packet_size);
  PvcGvcpPacket * gvcp_packet_new_read_memory_cmd (uint32_t address, uint32_t size, uint16_t packet_id, size_t *packet_size);
  PvcGvcpPacket * gvcp_packet_new_discovery_cmd (bool allow_broadcat_discovery_ack, size_t *packet_size);
  PvcGvcpPacket * gvcp_packet_new_write_register_cmd (uint32_t address, uint32_t value, uint16_t packet_id, size_t *packet_size);

  void waitForReceive();

  void send_complete (const sys::error_code& ec, size_t sz); 
  void DataReceive (const sys::error_code& ec, size_t sz, int num); 
  void DataReceive (const sys::error_code& ec, size_t sz); 

  UDPAsyncServer *m_stream_server;

private:
  asio::ip::udp::socket socket;
  asio::ip::udp::endpoint remote_peer;
  char buffer[MAXBUF];

  volatile std::sig_atomic_t gSignalStatus;
  int cmd_count;

};



