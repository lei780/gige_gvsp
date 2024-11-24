

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
#include <boost/atomic.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>


#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include "v4l2_driver.h"
#include "gvcp_header.h"

namespace asio = boost::asio;
namespace sys = boost::system;

const size_t MAXBUF = 256;

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



size_t arv_gvcp_packet_get_read_register_ack_size (void)
{
    return sizeof (ArvGvcpHeader) + sizeof (uint32_t);
}

ArvGvcpPacket *
arv_gvcp_packet_new_read_register_ack (uint32_t value,
                       uint16_t packet_id,
                       size_t *packet_size)
{
    ArvGvcpPacket *packet;
    uint32_t n_value = htonl (value);

    //g_return_val_if_fail (packet_size != NULL, NULL);

    *packet_size = arv_gvcp_packet_get_read_register_ack_size ();

    packet = (ArvGvcpPacket *)malloc (*packet_size);

    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_ACK;
    packet->header.packet_flags = 0;
    packet->header.command = htons (ARV_GVCP_COMMAND_READ_REGISTER_ACK);
    packet->header.size = htons (sizeof (uint32_t));
    packet->header.id = htons (packet_id);

    memcpy (&packet->data, &n_value, sizeof (uint32_t));

    return packet;
}

ArvGvcpPacket *
arv_gvcp_packet_new_read_register_cmd (uint32_t address,
                       uint16_t packet_id,
                       size_t *packet_size)
{
    ArvGvcpPacket *packet;
    uint32_t n_address = htonl (address);

    //g_return_val_if_fail (packet_size != NULL, NULL);

    *packet_size = sizeof (ArvGvcpHeader) + sizeof (uint32_t);

    packet = (ArvGvcpPacket *)malloc (*packet_size);

    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_CMD;
    packet->header.packet_flags = ARV_GVCP_CMD_PACKET_FLAGS_ACK_REQUIRED;
    packet->header.command = htons (ARV_GVCP_COMMAND_READ_REGISTER_CMD);
    packet->header.size = htons (sizeof (uint32_t));
    packet->header.id = htons (packet_id);

    memcpy (&packet->data, &n_address, sizeof (uint32_t));

    return packet;
}


ArvGvcpPacket *
arv_gvcp_packet_new_read_memory_cmd (uint32_t address, uint32_t size, uint16_t packet_id, size_t *packet_size)
{
    ArvGvcpPacket *packet;
    uint32_t n_address = htonl (address);
    uint32_t n_size;

    //g_return_val_if_fail (packet_size != NULL, NULL);

    n_size = htonl (((size + sizeof (uint32_t) - 1) / sizeof (uint32_t)) * sizeof (uint32_t));

    *packet_size = sizeof (ArvGvcpHeader) + 2 * sizeof (uint32_t);

    packet = (ArvGvcpPacket *)malloc (*packet_size);

    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_CMD;
    packet->header.packet_flags = ARV_GVCP_CMD_PACKET_FLAGS_ACK_REQUIRED;
    packet->header.command = htons (ARV_GVCP_COMMAND_READ_MEMORY_CMD);
    packet->header.size = htons (2 * sizeof (uint32_t));
    packet->header.id = htons (packet_id);

    memcpy (&packet->data, &n_address, sizeof (uint32_t));
    memcpy (&packet->data[sizeof(uint32_t)], &n_size, sizeof (uint32_t));

    return packet;
}

ArvGvcpPacket *
arv_gvcp_packet_new_discovery_cmd (bool allow_broadcat_discovery_ack, size_t *packet_size)
{
    ArvGvcpPacket *packet;

    //g_return_val_if_fail (packet_size != NULL, NULL);

    *packet_size = sizeof (ArvGvcpHeader);

    packet = (ArvGvcpPacket *)malloc (*packet_size);

    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_CMD;
    packet->header.packet_flags = ARV_GVCP_CMD_PACKET_FLAGS_ACK_REQUIRED |
                (allow_broadcat_discovery_ack ? ARV_GVCP_DISCOVERY_PACKET_FLAGS_ALLOW_BROADCAST_ACK : 0);
    packet->header.command = htons (ARV_GVCP_COMMAND_DISCOVERY_CMD);
    packet->header.size = 0x0000;
    packet->header.id = 0xffff;

    //packet->header.size = htons (0x0000);
    //packet->header.id = htons (0xffff);
    return packet;
}

ArvGvcpPacket *
arv_gvcp_packet_new_write_register_cmd (uint32_t address,
                    uint32_t value,
                    uint16_t packet_id,
                    size_t *packet_size)
{
    ArvGvcpPacket *packet;
    uint32_t n_address = htonl (address);
    uint32_t n_value = htonl (value);

    //g_return_val_if_fail (packet_size != NULL, NULL);

    *packet_size = sizeof (ArvGvcpHeader) + 2 * sizeof (uint32_t);

    packet = (ArvGvcpPacket *)malloc (*packet_size);

    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_CMD;
    packet->header.packet_flags = ARV_GVCP_CMD_PACKET_FLAGS_ACK_REQUIRED;
    packet->header.command = htons (ARV_GVCP_COMMAND_WRITE_REGISTER_CMD);
    packet->header.size = htons (2 * sizeof (uint32_t));
    packet->header.id = htons (packet_id);

    memcpy (&packet->data, &n_address, sizeof (uint32_t));
    memcpy (&packet->data[sizeof (uint32_t)], &n_value, sizeof (uint32_t));

    return packet;
}



class UDPAsyncCMDServer
{

public:
  UDPAsyncCMDServer(asio::io_service& service, unsigned short port)
     : socket(service,
          asio::ip::udp::endpoint(asio::ip::udp::v4(), port))
  {  waitForReceive();  }

  ~UDPAsyncCMDServer()
  {
      //std::cout << __FUNCTION__ << ": Exec.. " << std::endl;
#if BOOST_LOG_DYN_LINK == 1
      BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Exec..." << std::endl;
#endif
      socket.cancel();
  }

  void waitForReceive()
  {
      socket.async_receive_from(
           asio::buffer(buffer, MAXBUF),
           remote_peer,
           boost::bind(&UDPAsyncCMDServer::DataReceive, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred) );
  }

  void send_complete (const sys::error_code& ec, size_t sz)
  {
#if BOOST_LOG_DYN_LINK == 1
      BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << ": Exec..." << std::endl;
#endif
  }

  void DataReceive (const sys::error_code& ec, size_t sz)
  {
      //ArvGvcpPacket *gvcp_packet = (ArvGvcpPacket *)buffer;
      if( ec == boost::asio::error::operation_aborted )
      {
          std::cout << __FUNCTION__ << ": Aborted "  << '\n';
          return;
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

private:
  asio::ip::udp::socket socket;
  asio::ip::udp::endpoint remote_peer;
  char buffer[MAXBUF];

  volatile std::sig_atomic_t gSignalStatus;
};


int main(int argc, char *argv[]) 
{
  int loop = 0;
  int max_length = 1024;

  ArvGvcpPacket *cmd_packet;

  std::string gvcp_port = std::to_string(ARV_GVCP_PORT);

  if (argc < 2) {
    //std::cerr << "Usage: " << argv[0] << " host port\n";
    std::cerr << "Usage: " << argv[0] << " host \n";
    return 1;
  }

  asio::io_service service;
  try {

    //asio::ip::udp::resolver::query query(asio::ip::udp::v4(), argv[1], argv[2]);
    asio::ip::udp::resolver::query query(asio::ip::udp::v4(), argv[1], gvcp_port);
    asio::ip::udp::resolver resolver(service);

    auto iter = resolver.resolve(query);
    asio::ip::udp::endpoint endpoint = iter->endpoint();
    asio::ip::udp::socket socket(service, 
                                 asio::ip::udp::v4());

#if 0
    const char *msg = "Hello from client";
    socket.send_to( asio::buffer(msg, strlen(msg)), endpoint );

    char buffer[256];
    size_t recvd = socket.receive_from(asio::buffer(buffer,
                                 sizeof(buffer)), endpoint);
    buffer[recvd] = 0;
    std::cout << "Received [" << buffer << "] from " 
       << endpoint.address() << ':' << endpoint.port() << '\n';
#endif

    loop = 1;

    cmd_packet = nullptr;
    while(loop)
    {
        char request[max_length];
        size_t request_length;
        size_t buffer_size;

        std::cout << "Enter message: ";
        std::cin.getline(request, max_length);
        request_length = std::strlen(request);

        std::cout << "input [" << request << "]" << std::endl;

        if(request_length == 0) {
            std::cout << "no intput " << std::endl;
        }
        else if(std::strncmp(request, "quit", request_length) == 0) {
            std::cout << "loop = 0 " << std::endl;
            loop = 0;
        }
        else if(std::strncmp(request, "search", request_length) == 0) {

            cmd_packet = arv_gvcp_packet_new_discovery_cmd (true, &buffer_size);
            std::cout << "buffer_size: " << buffer_size << std::endl;
            if(cmd_packet != nullptr){
                socket.send_to( asio::buffer((char *)cmd_packet, buffer_size), endpoint );
                free(cmd_packet);
                cmd_packet = nullptr;
            }

        }
        else if(std::strncmp(request, "readport", request_length) == 0) {

            cmd_packet = arv_gvcp_packet_new_read_register_cmd (0x3008, 1, &buffer_size);
            std::cout << "buffer_size: " << buffer_size << std::endl;
            if(cmd_packet != nullptr){
                socket.send_to( asio::buffer((char *)cmd_packet, buffer_size), endpoint );
                free(cmd_packet);
                cmd_packet = nullptr;
            }

        }
        else if(std::strncmp(request, "start", request_length) == 0) {

            cmd_packet = arv_gvcp_packet_new_write_register_cmd (0x300C, 0x2, 2, &buffer_size);
            std::cout << "buffer_size: " << buffer_size << std::endl;
            if(cmd_packet != nullptr){
                socket.send_to( asio::buffer((char *)cmd_packet, buffer_size), endpoint );
                free(cmd_packet);
                cmd_packet = nullptr;
            }

        }
        else if(std::strncmp(request, "stop", request_length) == 0) {
            cmd_packet = arv_gvcp_packet_new_write_register_cmd (0x300C, 0x0, 2, &buffer_size);
            std::cout << "buffer_size: " << buffer_size << std::endl;
            if(cmd_packet != nullptr){
                socket.send_to( asio::buffer((char *)cmd_packet, buffer_size), endpoint );
                free(cmd_packet);
                cmd_packet = nullptr;
            }

        }
        else if(std::strncmp(request, "oneshot", request_length) == 0) {
            cmd_packet = arv_gvcp_packet_new_write_register_cmd (0x300C, 0x1, 2, &buffer_size);
            std::cout << "buffer_size: " << buffer_size << std::endl;
            if(cmd_packet != nullptr){
                socket.send_to( asio::buffer((char *)cmd_packet, buffer_size), endpoint );
                free(cmd_packet);
                cmd_packet = nullptr;
            }
        }
        else if(std::strncmp(request, "cont", request_length) == 0) {

        }
        else{
            std::cout << "do not found " << std::endl;
        }
    }

    //boost::asio::write(s, boost::asio::buffer(request, request_length));

  } catch (std::exception& e) {

    std::cerr << e.what() << '\n';

  }
}

