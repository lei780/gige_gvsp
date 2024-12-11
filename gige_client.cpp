
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

#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include "v4l2_driver.h"
#include "gvcp_header.h"

#include "yuv_output.h"

namespace asio = boost::asio;
namespace sys = boost::system;

const size_t MAXBUF = 1536;

#define ARV_GVSP_PACKET_EXTENDED_ID_MODE_MASK   0x80
#define ARV_GVSP_PACKET_ID_MASK         0x00ffffff
#define ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_MASK 0x7f000000
#define ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_POS  24
#define ARV_GVSP_PACKET_INFOS_N_PARTS_MASK      0x000000ff

 /* GVSP headers or IP + UDP + GVSP extended headers */

#define ARV_GVSP_PACKET_PROTOCOL_OVERHEAD(ext_ids)  ((ext_ids) ? \
                                                                 sizeof (ArvGvspPacket) +  sizeof (ArvGvspExtendedHeader) : \
                                                                 sizeof (ArvGvspPacket) +  sizeof (ArvGvspHeader))
#define ARV_GVSP_PAYLOAD_PACKET_PROTOCOL_OVERHEAD(ext_ids)      ARV_GVSP_PACKET_PROTOCOL_OVERHEAD(ext_ids)
#define ARV_GVSP_MULTIPART_PACKET_PROTOCOL_OVERHEAD(ext_ids)    ((ext_ids) ? \
                                                                 sizeof (ArvGvspPacket) + \
                                                                 sizeof (ArvGvspExtendedHeader) + \
                                                                 sizeof (ArvGvspMultipart) : \
                                                                 sizeof (ArvGvspPacket) + \
                                                                 sizeof (ArvGvspHeader) + \
                                                                 sizeof (ArvGvspMultipart))
/**
 * ArvGvspPacketType:
 * @ARV_GVSP_PACKET_TYPE_OK: valid packet
 * @ARV_GVSP_PACKET_TYPE_RESEND: resent packet (BlackFly PointGrey camera support)
 * @ARV_GVSP_PACKET_TYPE_PACKET_UNAVAILABLE: error packet, indicating invalid resend request
 */
typedef enum {
    ARV_GVSP_PACKET_TYPE_OK =     0x0000,
    ARV_GVSP_PACKET_TYPE_RESEND = 0x0100,
    ARV_GVSP_PACKET_TYPE_PACKET_UNAVAILABLE = 0x800c
} ArvGvspPacketType;

/**
 * ArvGvspContentType:
 * @ARV_GVSP_CONTENT_TYPE_LEADER: leader packet
 * @ARV_GVSP_CONTENT_TYPE_TRAILER: trailer packet
 * @ARV_GVSP_CONTENT_TYPE_PAYLOAD: data packet
 * @ARV_GVSP_CONTENT_TYPE_ALL_IN: leader + data + trailer packet
 * @ARV_GVSP_CONTENT_TYPE_H264: h264 data packet
 * @ARV_GVSP_CONTENT_TYPE_MULTIZONE: multizone data packet
 * @ARV_GVSP_CONTENT_TYPE_MULTIPART: multipart data packet
 * @ARV_GVSP_CONTENT_TYPE_GENDC: GenDC data packet
 */

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
    ARV_BUFFER_PAYLOAD_TYPE_CHUNK_DATA =    0x0004,
    ARV_BUFFER_PAYLOAD_TYPE_EXTENDED_CHUNK_DATA = 0x0005, /* Deprecated */
    ARV_BUFFER_PAYLOAD_TYPE_JPEG =          0x0006,
    ARV_BUFFER_PAYLOAD_TYPE_JPEG2000 =      0x0007,
    ARV_BUFFER_PAYLOAD_TYPE_H264 =          0x0008,
    ARV_BUFFER_PAYLOAD_TYPE_MULTIZONE_IMAGE =   0x0009,
    ARV_BUFFER_PAYLOAD_TYPE_MULTIPART =             0x000a,
    ARV_BUFFER_PAYLOAD_TYPE_GENDC_CONTAINER =       0x000b,
    ARV_BUFFER_PAYLOAD_TYPE_GENDC_COMPONENT_DATA =  0x000c
} ArvBufferPayloadType;


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


/**
 * ArvGvspLeader:
 * @flags: generic flags
 * @payload_type: ID of the payload type
 */

typedef struct {
    uint16_t flags;
    uint16_t payload_type;
    uint32_t timestamp_high;
    uint32_t timestamp_low;
} __attribute__((packed)) ArvGvspLeader;

typedef struct {
    uint16_t flags;
    uint16_t payload_type;
    uint32_t timestamp_high;
    uint32_t timestamp_low;
    ArvGvspImageInfos infos;
}  __attribute__((packed)) ArvGvspImageLeader;

typedef struct {
    uint8_t part_id;
    uint8_t zone_info;
    uint16_t offset_high;
    uint32_t offset_low;
} __attribute__((packed)) ArvGvspMultipart;


size_t arv_gvcp_packet_get_read_register_ack_size (void)
{
    return sizeof (ArvGvcpHeader) + sizeof (uint32_t);
}

ArvGvcpPacket * arv_gvcp_packet_new_read_register_ack (uint32_t value,
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

ArvGvcpPacket * arv_gvcp_packet_new_read_register_cmd (uint32_t address,
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

ArvGvcpPacket * arv_gvcp_packet_new_read_memory_cmd (uint32_t address, uint32_t size, uint16_t packet_id, size_t *packet_size)
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

ArvGvcpPacket * arv_gvcp_packet_new_discovery_cmd (bool allow_broadcat_discovery_ack, size_t *packet_size)
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

ArvGvcpPacket * arv_gvcp_packet_new_write_register_cmd (uint32_t address,
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

class UDPAsyncClient
{
public:
#if 0
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
#endif

  UDPAsyncClient(
	boost::asio::io_service& io_service, 
	const std::string& host, 
	const std::string& port ) 
     //: service(io_service), socket(io_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), 0)) {
     : socket(io_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), 0)) {

    std::cout << "ipaddress: "<< host << ", port: " << port << std::endl;
    //service = io_service;

	asio::ip::udp::resolver::query query(asio::ip::udp::v4(), host, port);
	asio::ip::udp::resolver resolver(io_service);
    auto iter = resolver.resolve(query);
	//asio::ip::udp::resolver::iterator iter = resolver.resolve(query);
    endpoint = iter->endpoint();
    //asio::ip::udp::socket socket(service, asio::ip::udp::v4(), 55000);
    //asio::ip::udp::socket socket(io_service, asio::ip::udp::v4());

    last_packet_id = 0;

    waitForReceive();
    //waitForReceive_1();

    const char *msg = "Connections";

    socket.async_send_to(
      asio::buffer(msg, strlen(msg)),
      endpoint,
      boost::bind(&UDPAsyncClient::send_complete, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred) );

     pyuv = nullptr;
     payload_count = 0;
  }

  ~UDPAsyncClient()
  {
    const char *msg = "Disconnections";

    socket.async_send_to(
      asio::buffer(msg, strlen(msg)),
      endpoint,
      boost::bind(&UDPAsyncClient::send_complete, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred) );

    socket.close();
  }

  ArvGvspPacketType arv_gvsp_packet_get_packet_type (const ArvGvspPacket *packet)
  {  
    return (ArvGvspPacketType) ntohs (packet->packet_type);
  }  
   
  bool arv_gvsp_packet_type_is_error (const ArvGvspPacketType packet_type)
  {
    return (packet_type & 0x8000) != 0;
  }   
    
  bool arv_gvsp_packet_has_extended_ids (const ArvGvspPacket *packet)
  {   
    return (packet->header[2] & ARV_GVSP_PACKET_EXTENDED_ID_MODE_MASK) != 0;
  }   

  ArvGvspContentType arv_gvsp_packet_get_content_type (const ArvGvspPacket *packet)
  {  
    if (arv_gvsp_packet_has_extended_ids (packet)) {
        ArvGvspExtendedHeader *header = (ArvGvspExtendedHeader *) &packet->header;

        return (ArvGvspContentType) ((ntohl (header->packet_infos) & ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_MASK) >>
                         ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_POS);
    } else {
        ArvGvspHeader *header = (ArvGvspHeader *) &packet->header;

        return (ArvGvspContentType) ((ntohl (header->packet_infos) & ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_MASK) >>
                         ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_POS);
    }
  }

  uint32_t arv_gvsp_packet_get_packet_id (const ArvGvspPacket *packet)
  {
    if (arv_gvsp_packet_has_extended_ids (packet)) {
        ArvGvspExtendedHeader *header = (ArvGvspExtendedHeader *) &packet->header;

        return ntohl (header->packet_id);
    } else {
        ArvGvspHeader *header = (ArvGvspHeader *) &packet->header;

        return ntohl (header->packet_infos) & ARV_GVSP_PACKET_ID_MASK;
    }
  }

  uint64_t arv_gvsp_packet_get_frame_id (const ArvGvspPacket *packet)
  {
    if (arv_gvsp_packet_has_extended_ids (packet)) {
        ArvGvspExtendedHeader *header = (ArvGvspExtendedHeader *) &packet->header;

        //return GUINT64_FROM_BE(header->frame_id);
        return header->frame_id;
    } else {
        ArvGvspHeader *header = (ArvGvspHeader *) &packet->header;

        return ntohs (header->frame_id);
    }
  }

  void * arv_gvsp_packet_get_data (const ArvGvspPacket *packet)
  {
    if (arv_gvsp_packet_has_extended_ids (packet)) {
        ArvGvspExtendedHeader *header = (ArvGvspExtendedHeader *) &packet->header;

        return &header->data;
    } else {
        ArvGvspHeader *header = (ArvGvspHeader *) &packet->header;

        return &header->data;
    }
  }

  ArvBufferPayloadType
  arv_gvsp_leader_packet_get_buffer_payload_type (const ArvGvspPacket *packet, bool *has_chunks)
  {   
    if ( arv_gvsp_packet_get_content_type (packet) == ARV_GVSP_CONTENT_TYPE_LEADER ) {
        ArvGvspLeader *leader;
        uint16_t payload_type;

        leader = (ArvGvspLeader *) arv_gvsp_packet_get_data (packet);
        payload_type = ntohs (leader->payload_type);
        
        if (has_chunks != NULL)
            *has_chunks = ( (payload_type & 0x4000) != 0 ||
                            (payload_type == ARV_BUFFER_PAYLOAD_TYPE_CHUNK_DATA) ||
                            (payload_type == ARV_BUFFER_PAYLOAD_TYPE_EXTENDED_CHUNK_DATA));

        return (ArvBufferPayloadType) (payload_type & 0x3fff);
    }

    return ARV_BUFFER_PAYLOAD_TYPE_UNKNOWN;
  }

  void waitForReceive()
  {
      char *pbuf = nullptr;
      int flag = 0;

      pbuf = new char[MAXBUF];
      
      if(pbuf != nullptr){
          socket.async_receive_from(
              asio::buffer(pbuf, MAXBUF),
              endpoint,
              boost::bind(&UDPAsyncClient::DataReceive, this, _1, _2, flag, pbuf) );
      }
      else{
          flag = 1;
      }
  }

  //void waitForReceive_1()
  //{
  //    socket.async_receive_from(
  //         asio::buffer(buffer_1, MAXBUF),
  //         endpoint,
  //         boost::bind(&UDPAsyncClient::DataReceive, this,
  //              boost::asio::placeholders::error,
  //              boost::asio::placeholders::bytes_transferred, 1) );
  //}

  void send_complete (const sys::error_code& ec, size_t sz)
  {
      std::cout << __FUNCTION__ << ": Exec..." << std::endl;
  }

  //void DataReceive (const sys::error_code& ec, size_t sz, int num)
  void DataReceive (const sys::error_code& ec, size_t sz, int num, char *ptr)
  {
      ArvGvspPacket *gvsp_packet;
      ArvGvspContentType content_type;
      ArvBufferPayloadType payload_type;
      uint32_t packet_id;
      uint64_t frame_id; 
      bool extended_ids;
      size_t packet_size = sz;
      uint32_t block_size = 0;

      if( ec == boost::asio::error::operation_aborted )
      {
          std::cout << __FUNCTION__ << ": Aborted "  << '\n';
          return;
      }

      gvsp_packet = (ArvGvspPacket *)ptr;
      //if(num == 0)
      //   gvsp_packet = (ArvGvspPacket *)buffer;
      //else
      //    gvsp_packet = (ArvGvspPacket *)buffer_1;

      waitForReceive();

      frame_id = arv_gvsp_packet_get_frame_id (gvsp_packet);
      packet_id = arv_gvsp_packet_get_packet_id (gvsp_packet);

      //BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << "_" << num << ": received size = " << packet_size << '\n';
      //std::cout << __FUNCTION__ << ": packet id = " << packet_id <<" frame id = " << frame_id << '\n';

      if((packet_id != 0) && (packet_id - last_packet_id != 1)){
          std::cout << __FUNCTION__ << ": current packet id = " << packet_id <<" frame id = " << frame_id << '\n';
          std::cout << __FUNCTION__ << "_" << num << ": last packet id = " << last_packet_id << '\n';
      }

      last_packet_id = packet_id;
      //BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << "_" << num << ": last packet id = " << last_packet_id << '\n';

      extended_ids = arv_gvsp_packet_has_extended_ids (gvsp_packet);
      content_type = arv_gvsp_packet_get_content_type (gvsp_packet);

      switch (content_type) {

          case ARV_GVSP_CONTENT_TYPE_LEADER:
              payload_type = arv_gvsp_leader_packet_get_buffer_payload_type(gvsp_packet, NULL);
              if ( payload_type == ARV_BUFFER_PAYLOAD_TYPE_IMAGE ||
                   payload_type == ARV_BUFFER_PAYLOAD_TYPE_EXTENDED_CHUNK_DATA ||
                   payload_type == ARV_BUFFER_PAYLOAD_TYPE_CHUNK_DATA ) {

                  block_size = packet_size - ARV_GVSP_PAYLOAD_PACKET_PROTOCOL_OVERHEAD (extended_ids);
              }
              //std::cout << __FUNCTION__ << "ARV_GVSP_CONTENT_TYPE_LEADER (block_size=" << block_size << ") " << std::endl;
              //BOOST_LOG_TRIVIAL(info) << "ARV_GVSP_CONTENT_TYPE_LEADER allocated buffer ";
              pyuv = new char[1920*1080*2];
              payload_count = 0; 
              break;

          case ARV_GVSP_CONTENT_TYPE_PAYLOAD:
              block_size = packet_size - ARV_GVSP_PAYLOAD_PACKET_PROTOCOL_OVERHEAD (extended_ids);
              if(pyuv != nullptr){
                  //BOOST_LOG_TRIVIAL(info) << " ARV_GVSP_CONTENT_TYPE_PAYLOAD copyed " << payload_count;
                  //memcpy( pyuv+(payload_count*block_size), ((char *)gvsp_packet)+ARV_GVSP_PAYLOAD_PACKET_PROTOCOL_OVERHEAD(extended_ids), block_size);  
                  memcpy( pyuv+(payload_count*block_size), ((char *)gvsp_packet)+8, block_size);  
                  payload_count++;
              }
              //std::cout << __FUNCTION__ << "ARV_GVSP_CONTENT_TYPE_PAYLOAD (block_size=" << block_size << ") " << std::endl;
              //return (allocated_size + block_size - 1) / block_size + (2 /* leader + trailer */);
              break;

          case ARV_GVSP_CONTENT_TYPE_MULTIPART:
              block_size = packet_size - ARV_GVSP_MULTIPART_PACKET_PROTOCOL_OVERHEAD (extended_ids);
              //return (allocated_size + block_size - 1) / block_size +
              //                 (2 /* leader + trailer */) + (255 /* n_parts_max) */);
              break;

          case ARV_GVSP_CONTENT_TYPE_TRAILER:
              //std::cout << __FUNCTION__ << "ARV_GVSP_CONTENT_TYPE_TRAILER " << std::endl;
              //BOOST_LOG_TRIVIAL(info) << "ARV_GVSP_CONTENT_TYPE_TRAILER  " << payload_count;
              last_packet_id = 0;

              feeding_data( pyuv, 1480*(payload_count));

              //return arv_gvsp_packet_get_packet_id (packet) + 1;
              break;

          case ARV_GVSP_CONTENT_TYPE_ALL_IN:
              //return 1;
              break;

          case ARV_GVSP_CONTENT_TYPE_H264:
          case ARV_GVSP_CONTENT_TYPE_GENDC:
          case ARV_GVSP_CONTENT_TYPE_MULTIZONE:
              break;
     }
     delete[] ptr;

     //waitForReceive();
  }

  void signal_handler(int signal)
  {
      gSignalStatus = signal;
      socket.cancel();
      socket.close();
  }

private:
  asio::ip::udp::socket socket;
  asio::ip::udp::endpoint remote_peer;
  asio::ip::udp::endpoint endpoint;
  boost::asio::ip::udp::endpoint endpoint_;
  volatile std::sig_atomic_t gSignalStatus;
  boost::atomic <uint32_t> last_packet_id;
  char *pyuv ;
  char buffer[1536] ;
  uint32_t payload_count;
};

UDPAsyncClient *stream_client = nullptr;

void stream_proc(void *pdat)
{
    asio::io_service *srv = (asio::io_service *)pdat;
    srv->run();
}

int main(int argc, char *argv[]) 
{
  int loop = 0;
  int max_length = 1024;

  ArvGvcpPacket *cmd_packet;

  std::string gvcp_port = std::to_string(ARV_GVCP_PORT);
  std::string gvsp_port = std::to_string(55000);

  if (argc < 2) {
    //std::cerr << "Usage: " << argv[0] << " host port\n";
    std::cerr << "Usage: " << argv[0] << " host \n";
    return 1;
  }

  asio::io_service stream_service;
  stream_client = new UDPAsyncClient(stream_service, argv[1], gvsp_port);

  std::function<void()> func = std::bind(&stream_proc, &stream_service);
  std::thread my_thread(func);

  //std::function<void()> func_1 = std::bind(&output_main);
  //std::thread my_thread_1(func_1);
  std::thread my_thread_1(output_main);

  asio::io_service service;

  try {

    //asio::ip::udp::resolver::query query(asio::ip::udp::v4(), argv[1], argv[2]);
    asio::ip::udp::resolver::query query(asio::ip::udp::v4(), argv[1], gvcp_port);
    asio::ip::udp::resolver resolver(service);

    auto iter = resolver.resolve(query);
    asio::ip::udp::endpoint endpoint = iter->endpoint();
    asio::ip::udp::socket socket(service, asio::ip::udp::v4());

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

  //stream_client->signal_handler(9);
  delete stream_client;

  output_main_stop() ;

  my_thread.join();
  my_thread_1.join(); 

}

