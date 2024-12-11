
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

#include "gev_gvcp.h"

namespace asio = boost::asio;
namespace sys = boost::system;
using namespace boost::placeholders;

const size_t MAXBUF = 256;

//UDPAsyncCMDServer::UDPAsyncCMDServer(asio::io_service& service, unsigned short port) 
//   : socket(service, 
//        asio::ip::udp::endpoint(asio::ip::udp::v4(), port))
//{  
//  cmd_count = 0;
//  waitForReceive();  
//}

UDPAsyncCMDServer::~UDPAsyncCMDServer() 
{ 
    std::cout << __FUNCTION__ << ": Exec..." << std::endl; 
    socket.cancel();
}

PvcGvcpPacketType UDPAsyncCMDServer::gvcp_packet_get_packet_type (PvcGvcpPacket *packet)
{   
  if (packet == NULL)
      return ARV_GVCP_PACKET_TYPE_ERROR;

  return (PvcGvcpPacketType) packet->header.packet_type;
}

PvcGvcpCommand UDPAsyncCMDServer::gvcp_packet_get_command (PvcGvcpPacket *packet)
{
  if (packet == NULL)
      return (PvcGvcpCommand) 0;

  return (PvcGvcpCommand) ntohs (packet->header.command);
}

uint16_t UDPAsyncCMDServer::gvcp_packet_get_packet_id (PvcGvcpPacket *packet)
{
  if (packet == NULL)
      return 0;

  return ntohs (packet->header.id);
}

void UDPAsyncCMDServer::gvcp_packet_get_read_memory_cmd_infos (const PvcGvcpPacket *packet, uint32_t *address, uint32_t *size)
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
      *address = ntohl (*((uint32_t *) ((char *) packet + sizeof (PvcGvcpPacket))));

  if (size != NULL)
      *size = (ntohl (*((uint32_t *) ((char *) packet + sizeof (PvcGvcpPacket) + sizeof (uint32_t))))) & 0xffff;
}


PvcGvcpPacket *
UDPAsyncCMDServer::gvcp_packet_new_discovery_ack (uint16_t packet_id, size_t *packet_size)
{   
  PvcGvcpPacket *packet;
  
  //g_return_val_if_fail (packet_size != NULL, NULL);
      
  *packet_size = sizeof (PvcGvcpHeader) + ARV_GVBS_DISCOVERY_DATA_SIZE ;

  packet = (PvcGvcpPacket *)malloc (*packet_size);
      
  packet->header.packet_type = ARV_GVCP_PACKET_TYPE_ACK; 
  packet->header.packet_flags = 0;
  packet->header.command = htons (ARV_GVCP_COMMAND_DISCOVERY_ACK); 
  packet->header.size = htons (ARV_GVBS_DISCOVERY_DATA_SIZE);
  packet->header.id = htons (packet_id);

  return packet;
}    

size_t UDPAsyncCMDServer::gvcp_packet_get_read_register_ack_size (void)
{
  return sizeof (PvcGvcpHeader) + sizeof (uint32_t); 
}

void UDPAsyncCMDServer::gvcp_packet_get_write_register_cmd_infos (const PvcGvcpPacket *packet, uint32_t *address, uint32_t *value)
{
  if (packet == NULL) {
      if (address != NULL)
          *address = 0;

      if (value != NULL)
          *value = 0;

      return;
  }

  if (address != NULL)
      *address = ntohl (*((uint32_t *) ((char *) packet + sizeof (PvcGvcpPacket))));

  if (value != NULL)
      *value = ntohl (*((uint32_t *) ((char *) packet + sizeof (PvcGvcpPacket) + sizeof (uint32_t))));
}


void UDPAsyncCMDServer::gvcp_packet_get_read_register_cmd_infos (const PvcGvcpPacket *packet, uint32_t *address)
{
  if (packet == NULL) {
      if (address != NULL)
          *address = 0;
      return;
  }

  if (address != NULL) 
      *address = ntohl (*((uint32_t *) ((char *) packet + sizeof (PvcGvcpPacket))));
}

uint32_t UDPAsyncCMDServer::gvcp_packet_get_read_register_ack_value (const PvcGvcpPacket *packet)
{
  if (packet == NULL)
      return 0;
  return ntohl (*((uint32_t *) ((char *) packet + sizeof (PvcGvcpPacket))));
}

PvcGvcpPacket * UDPAsyncCMDServer::gvcp_packet_new_read_register_ack (uint32_t value,
                       uint16_t packet_id,
                       size_t *packet_size)
{
    PvcGvcpPacket *packet;
    uint32_t n_value = htonl (value);

    //g_return_val_if_fail (packet_size != NULL, NULL);

    *packet_size = gvcp_packet_get_read_register_ack_size ();

    packet = (PvcGvcpPacket *)malloc (*packet_size);

    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_ACK;
    packet->header.packet_flags = 0;
    packet->header.command = htons (ARV_GVCP_COMMAND_READ_REGISTER_ACK);
    packet->header.size = htons (sizeof (uint32_t));
    packet->header.id = htons (packet_id);

    memcpy (&packet->data, &n_value, sizeof (uint32_t));

    return packet;
}


PvcGvcpPacket * UDPAsyncCMDServer::gvcp_packet_new_read_register_cmd (uint32_t address,
                       uint16_t packet_id,
                       size_t *packet_size)
{
    PvcGvcpPacket *packet;
    uint32_t n_address = htonl (address);

    //g_return_val_if_fail (packet_size != NULL, NULL);

    *packet_size = sizeof (PvcGvcpHeader) + sizeof (uint32_t);

    packet = (PvcGvcpPacket *)malloc (*packet_size);

    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_CMD;
    packet->header.packet_flags = ARV_GVCP_CMD_PACKET_FLAGS_ACK_REQUIRED;
    packet->header.command = htons (ARV_GVCP_COMMAND_READ_REGISTER_CMD);
    packet->header.size = htons (sizeof (uint32_t));
    packet->header.id = htons (packet_id);

    memcpy (&packet->data, &n_address, sizeof (uint32_t));

    return packet;
}

PvcGvcpPacket * UDPAsyncCMDServer::gvcp_packet_new_read_memory_cmd (uint32_t address, uint32_t size, uint16_t packet_id, size_t *packet_size)
{
    PvcGvcpPacket *packet;
    uint32_t n_address = htonl (address);
    uint32_t n_size;

    //g_return_val_if_fail (packet_size != NULL, NULL);

    n_size = htonl (((size + sizeof (uint32_t) - 1) / sizeof (uint32_t)) * sizeof (uint32_t));
    *packet_size = sizeof (PvcGvcpHeader) + 2 * sizeof (uint32_t);

    packet = (PvcGvcpPacket *)malloc (*packet_size);

    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_CMD;
    packet->header.packet_flags = ARV_GVCP_CMD_PACKET_FLAGS_ACK_REQUIRED;
    packet->header.command = htons (ARV_GVCP_COMMAND_READ_MEMORY_CMD);
    packet->header.size = htons (2 * sizeof (uint32_t));
    packet->header.id = htons (packet_id);

    memcpy (&packet->data, &n_address, sizeof (uint32_t));
    memcpy (&packet->data[sizeof(uint32_t)], &n_size, sizeof (uint32_t));

    return packet;
}


PvcGvcpPacket * UDPAsyncCMDServer::gvcp_packet_new_discovery_cmd (bool allow_broadcat_discovery_ack, size_t *packet_size)
{
    PvcGvcpPacket *packet;

    //g_return_val_if_fail (packet_size != NULL, NULL);

    *packet_size = sizeof (PvcGvcpHeader);

    packet = (PvcGvcpPacket *)malloc (*packet_size);

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


PvcGvcpPacket * UDPAsyncCMDServer::gvcp_packet_new_write_register_cmd (uint32_t address,
                    uint32_t value,
                    uint16_t packet_id,
                    size_t *packet_size)
{
    PvcGvcpPacket *packet;
    uint32_t n_address = htonl (address);
    uint32_t n_value = htonl (value);

    //g_return_val_if_fail (packet_size != NULL, NULL);

    *packet_size = sizeof (PvcGvcpHeader) + 2 * sizeof (uint32_t);

    packet = (PvcGvcpPacket *)malloc (*packet_size);

    packet->header.packet_type = ARV_GVCP_PACKET_TYPE_CMD;
    packet->header.packet_flags = ARV_GVCP_CMD_PACKET_FLAGS_ACK_REQUIRED;
    packet->header.command = htons (ARV_GVCP_COMMAND_WRITE_REGISTER_CMD);
    packet->header.size = htons (2 * sizeof (uint32_t));
    packet->header.id = htons (packet_id);

    memcpy (&packet->data, &n_address, sizeof (uint32_t));
    memcpy (&packet->data[sizeof (uint32_t)], &n_value, sizeof (uint32_t));

    return packet;
}



bool UDPAsyncCMDServer::gev_camera_read_memory (uint32_t address, uint32_t size, void *buffer)
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

void UDPAsyncCMDServer::waitForReceive() 
{
    //std::cout<<"Listening at : " << socket.local_endpoint() << std::endl;
    socket.async_receive_from( 
         asio::buffer(buffer, MAXBUF), 
         remote_peer,
         boost::bind(&UDPAsyncCMDServer::DataReceive, this, _1, _2) );
}

void UDPAsyncCMDServer::send_complete (const sys::error_code& ec, size_t sz) 
{
    //std::cout << __FUNCTION__ << ": Exec..." << std::endl; 
}

void UDPAsyncCMDServer::DataReceive (const sys::error_code& ec, size_t sz, int num) 
{
    //PvcGvcpPacket *gvcp_packet = (PvcGvcpPacket *)buffer;

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

void UDPAsyncCMDServer::DataReceive (const sys::error_code& ec, size_t sz) 
{
    //const char *msg = "hello from server";
    PvcGvcpPacket *gvcp_packet = (PvcGvcpPacket *)buffer;

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
          std::cout <<"["<<__FUNCTION__<<" "<< __LINE__<<"] "<<"GVCP_Discovery Command " << std::endl;

          //ack_packet = arv_gvcp_packet_new_discovery_ack (packet_id, &ack_packet_size);
          //gev_camera_read_memory (0, ARV_GVBS_DISCOVERY_DATA_SIZE, &ack_packet->data);

          break;

      case ARV_GVCP_COMMAND_READ_MEMORY_CMD:
          std::cout <<"["<<__FUNCTION__<<"] "<<"GVCP_ReadMemory Command " << std::endl;
          break;

      case ARV_GVCP_COMMAND_WRITE_MEMORY_CMD:
          std::cout <<"["<<__FUNCTION__<<"] "<<"GVCP_WriteMemory Command " << std::endl;
          break;

      case ARV_GVCP_COMMAND_READ_REGISTER_CMD:

          std::cout <<"["<<__FUNCTION__<<"] "<<"GVCP_ReadRegister Command " << std::endl;

          gvcp_packet_get_read_register_cmd_infos (gvcp_packet, &register_address);
          std::cout <<"["<<__FUNCTION__<<"] "<< "Read register command 0x" << std::hex << register_address ;

          success = gev_camera_read_memory (register_address, sizeof (register_value), &be_value);
          //          register_address, register_value);
          //ack_packet = gvcp_packet_new_read_register_ack (register_value, packet_id,
          //                            &ack_packet_size);
          ((void)success);
          break;

      case ARV_GVCP_COMMAND_WRITE_REGISTER_CMD:
          //uint32_t register_address;
          //uint32_t register_value;

          register_value = 0;
          std::cout <<"["<<__FUNCTION__<<"] "<<"GVCP_WriteRegister Command " << std::endl;
          gvcp_packet_get_write_register_cmd_infos (gvcp_packet, &register_address, &register_value);

          std::cout <<"["<<__FUNCTION__<<"] "<< "Read register address = 0x" << std::hex << register_address << std::endl ;
          std::cout <<"["<<__FUNCTION__<<"] "<< "Read register value = " << std::hex << register_value << std::endl ;

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

          break;

     default:
          std::cout <<"["<<__FUNCTION__<<"] "<<"Unknown command" << std::endl;

    }

    waitForReceive();
}

//  void signal_handler(int signal)
//  {
//      gSignalStatus = signal;
//      socket.cancel();
//  }



