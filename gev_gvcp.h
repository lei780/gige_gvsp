
#ifndef __GVCP_HEADER_H__
#define __GVCP_HEADER_H__

#include "stream_gev.h"

/**
 * ARV_GVCP_PORT:
 *
 * Standard device listening port for GVCP packets
 */
#define ARV_GVCP_PORT   3956

#define ARV_GVBS_VERSION_OFFSET             0x00000000
#define ARV_GVBS_VERSION_MINOR_MASK         0x0000ffff
#define ARV_GVBS_VERSION_MINOR_POS          0
#define ARV_GVBS_VERSION_MAJOR_MASK         0xffff0000
#define ARV_GVBS_VERSION_MAJOR_POS          16

#define ARV_GVBS_DEVICE_MODE_OFFSET         0x00000004
#define ARV_GVBS_DEVICE_MODE_BIG_ENDIAN         1 << 31
#define ARV_GVBS_DEVICE_MODE_CHARACTER_SET_MASK     0x0000ffff
#define ARV_GVBS_DEVICE_MODE_CHARACTER_SET_POS      0

#define ARV_GVBS_DEVICE_MAC_ADDRESS_HIGH_OFFSET     0x00000008
#define ARV_GVBS_DEVICE_MAC_ADDRESS_LOW_OFFSET      0x0000000c

#define ARV_GVBS_SUPPORTED_IP_CONFIGURATION_OFFSET  0x00000010
#define ARV_GVBS_CURRENT_IP_CONFIGURATION_OFFSET    0x00000014
#define ARV_GVBS_IP_CONFIGURATION_PERSISTENT        1 << 0
#define ARV_GVBS_IP_CONFIGURATION_DHCP          1 << 1
#define ARV_GVBS_IP_CONFIGURATION_LLA           1 << 2

#define ARV_GVBS_CURRENT_IP_ADDRESS_OFFSET      0x00000024
#define ARV_GVBS_CURRENT_SUBNET_MASK_OFFSET     0x00000034
#define ARV_GVBS_CURRENT_GATEWAY_OFFSET         0x00000044

#define ARV_GVBS_MANUFACTURER_NAME_OFFSET       0x00000048
#define ARV_GVBS_MANUFACTURER_NAME_SIZE         32

#define ARV_GVBS_MODEL_NAME_OFFSET          0x00000068
#define ARV_GVBS_MODEL_NAME_SIZE            32

#define ARV_GVBS_DEVICE_VERSION_OFFSET          0x00000088
#define ARV_GVBS_DEVICE_VERSION_SIZE            32

#define ARV_GVBS_MANUFACTURER_INFO_OFFSET           0x000000a8
#define ARV_GVBS_MANUFACTURER_INFO_SIZE             48

#define ARV_GVBS_SERIAL_NUMBER_OFFSET           0x000000d8
#define ARV_GVBS_SERIAL_NUMBER_SIZE         16

#define ARV_GVBS_USER_DEFINED_NAME_OFFSET       0x000000e8
#define ARV_GVBS_USER_DEFINED_NAME_SIZE         16

#define ARV_GVBS_DISCOVERY_DATA_SIZE            0xf8

#define ARV_GVBS_XML_URL_0_OFFSET           0x00000200
#define ARV_GVBS_XML_URL_1_OFFSET           0x00000400
#define ARV_GVBS_XML_URL_SIZE               512

#define ARV_GVBS_N_NETWORK_INTERFACES_OFFSET        0x00000600

#define ARV_GVBS_PERSISTENT_IP_ADDRESS_0_OFFSET     0x0000064c
#define ARV_GVBS_PERSISTENT_SUBNET_MASK_0_OFFSET    0x0000065c
#define ARV_GVBS_PERSISTENT_GATEWAY_0_OFFSET        0x0000066c

#define ARV_GVBS_N_MESSAGE_CHANNELS_OFFSET      0x00000900
#define ARV_GVBS_N_STREAM_CHANNELS_OFFSET       0x00000904

#define ARV_GVBS_GVCP_CAPABILITY_OFFSET         0x00000934
#define ARV_GVBS_GVCP_CAPABILITY_CONCATENATION      1 << 0
#define ARV_GVBS_GVCP_CAPABILITY_WRITE_MEMORY       1 << 1
#define ARV_GVBS_GVCP_CAPABILITY_PACKET_RESEND      1 << 2
#define ARV_GVBS_GVCP_CAPABILITY_EVENT              1 << 3
#define ARV_GVBS_GVCP_CAPABILITY_EVENT_DATA         1 << 4
#define ARV_GVBS_GVCP_CAPABILITY_PENDING_ACK        1 << 5
#define ARV_GVBS_GVCP_CAPABILITY_ACTION             1 << 6
#define ARV_GVBS_GVCP_CAPABILITY_PRIMARY_APPLICATION_SWITCHOVER 1 << 21
#define ARV_GVBS_GVCP_CAPABILITY_EXTENDED_STATUS_CODES          1 << 22
#define ARV_GVBS_GVCP_CAPABILITY_DISCOVERY_ACK_DELAY_WRITABLE   1 << 23
#define ARV_GVBS_GVCP_CAPABILITY_DISCOVERY_ACK_DELAY    1 << 24
#define ARV_GVBS_GVCP_CAPABILITY_TEST_DATA              1 << 25
#define ARV_GVBS_GVCP_CAPABILITY_MANIFEST_TABLE         1 << 26
#define ARV_GVBS_GVCP_CAPABILITY_CCP_APPLICATION_SOCKET 1 << 27
#define ARV_GVBS_GVCP_CAPABILITY_LINK_SPEED         1 << 28
#define ARV_GVBS_GVCP_CAPABILITY_HEARTBEAT_DISABLE  1 << 29
#define ARV_GVBS_GVCP_CAPABILITY_SERIAL_NUMBER      1 << 30
#define ARV_GVBS_GVCP_CAPABILITY_NAME_REGISTER      1 << 31

#define ARV_GVBS_HEARTBEAT_TIMEOUT_OFFSET       0x00000938
#define ARV_GVBS_TIMESTAMP_TICK_FREQUENCY_HIGH_OFFSET   0x0000093c
#define ARV_GVBS_TIMESTAMP_TICK_FREQUENCY_LOW_OFFSET    0x00000940
#define ARV_GVBS_TIMESTAMP_CONTROL_OFFSET            0x00000944
#define ARV_GVBS_TIMESTAMP_LATCHED_VALUE_HIGH_OFFSET 0x00000948
#define ARV_GVBS_TIMESTAMP_LATCHED_VALUE_LOW_OFFSET  0x0000094c

#define ARV_GVBS_CONTROL_CHANNEL_PRIVILEGE_OFFSET   0x00000a00
#define ARV_GVBS_CONTROL_CHANNEL_PRIVILEGE_CONTROL  1 << 1
#define ARV_GVBS_CONTROL_CHANNEL_PRIVILEGE_EXCLUSIVE    1 << 0

#define ARV_GVBS_STREAM_CHANNEL_0_PORT_OFFSET       0x00000d00

#define ARV_GVBS_STREAM_CHANNEL_0_PACKET_SIZE_OFFSET    0x00000d04
#define ARV_GVBS_STREAM_CHANNEL_0_PACKET_SIZE_MASK      0x0000ffff
#define ARV_GVBS_STREAM_CHANNEL_0_PACKET_SIZE_POS       0
#define ARV_GVBS_STREAM_CHANNEL_0_PACKET_BIG_ENDIAN     1 << 29
#define ARV_GVBS_STREAM_CHANNEL_0_PACKET_DO_NOT_FRAGMENT    1 << 30
#define ARV_GVBS_STREAM_CHANNEL_0_PACKET_SIZE_FIRE_TEST     1 << 31

#define ARV_GVBS_STREAM_CHANNEL_0_PACKET_DELAY_OFFSET   0x00000d08
#define ARV_GVBS_STREAM_CHANNEL_0_IP_ADDRESS_OFFSET     0x00000d18

#define ARV_GVCP_DATA_SIZE_MAX              512



/**
 * ArvGvcpPacketType:
 * @ARV_GVCP_PACKET_TYPE_ACK: acknowledge packet
 * @ARV_GVCP_PACKET_TYPE_CMD: command packet
 * @ARV_GVCP_PACKET_TYPE_ERROR: error packet
 * @ARV_GVCP_PACKET_TYPE_UNKNOWN_ERROR: unknown error
 */

typedef enum {
    ARV_GVCP_PACKET_TYPE_ACK =      0x00,
    ARV_GVCP_PACKET_TYPE_CMD =      0x42,
    ARV_GVCP_PACKET_TYPE_ERROR =        0x80,
    ARV_GVCP_PACKET_TYPE_UNKNOWN_ERROR =    0x8f
} PvcGvcpPacketType;


/**
 * ArvGvcpError:
 * @ARV_GVCP_ERROR_NONE: none
 * @ARV_GVCP_ERROR_NOT_IMPLEMENTED: not implemented
 * @ARV_GVCP_ERROR_INVALID_PARAMETER: invalid parameter
 * @ARV_GVCP_ERROR_INVALID_ACCESS: inavlid access
 * @ARV_GVCP_ERROR_WRITE_PROTECT: write protect
 * @ARV_GVCP_ERROR_BAD_ALIGNMENT: bad alignment
 * @ARV_GVCP_ERROR_ACCESS_DENIED: access denied
 * @ARV_GVCP_ERROR_BUSY: busy
 * @ARV_GVCP_ERROR_LOCAL_PROBLEM: local problem
 * @ARV_GVCP_ERROR_MESSAGE_MISMATCH: message mismatch
 * @ARV_GVCP_ERROR_INVALID_PROTOCOL: invalid protocol
 * @ARV_GVCP_ERROR_NO_MESSAGE: no message
 * @ARV_GVCP_ERROR_PACKET_UNAVAILABLE: packet unavailable
 * @ARV_GVCP_ERROR_DATA_OVERRUN: data overrun
 * @ARV_GVCP_ERROR_INVALID_HEADER: invalid header
 * @ARV_GVCP_ERROR_WRONG_CONFIG: wrong config
 * @ARV_GVCP_ERROR_PACKET_NOT_YET_AVAILABLE: packet not yet available
 * @ARV_GVCP_ERROR_PACKET_AND_PREVIOUS_REMOVED_FROM_MEMORY: packet and previous removed from memmory
 * @ARV_GVCP_ERROR_PACKET__REMOVED_FROM_MEMORY: packet removed from memory
 * @ARV_GVCP_ERROR_NO_REFERENCE_TIME: no reference time
 * @ARV_GVCP_ERROR_PACKET_TEMPORARILY_UNAVAILABLE: packet temporarily unavailable
 * @ARV_GVCP_ERROR_OVERFLOW: overflow
 * @ARV_GVCP_ERROR_ACTION_LATE: action late
 * @ARV_GVCP_ERROR_LEADER_TRAILER_OVERFLOW: leader trailer overflow
 */

typedef enum {
    ARV_GVCP_ERROR_NONE =                       0x00,
    ARV_GVCP_ERROR_NOT_IMPLEMENTED =            0x01,
    ARV_GVCP_ERROR_INVALID_PARAMETER =          0x02,
    ARV_GVCP_ERROR_INVALID_ACCESS =             0x03,
    ARV_GVCP_ERROR_WRITE_PROTECT =              0x04,
    ARV_GVCP_ERROR_BAD_ALIGNMENT =              0x05,
    ARV_GVCP_ERROR_ACCESS_DENIED =              0x06,
    ARV_GVCP_ERROR_BUSY =                       0x07,
    ARV_GVCP_ERROR_LOCAL_PROBLEM =              0x08,
    ARV_GVCP_ERROR_MESSAGE_MISMATCH =           0x09,
    ARV_GVCP_ERROR_INVALID_PROTOCOL =           0x0a,
    ARV_GVCP_ERROR_NO_MESSAGE =                 0x0b,
    ARV_GVCP_ERROR_PACKET_UNAVAILABLE =         0x0c,
    ARV_GVCP_ERROR_DATA_OVERRUN =               0x0d,
    ARV_GVCP_ERROR_INVALID_HEADER =             0x0e,
    ARV_GVCP_ERROR_WRONG_CONFIG =               0x0f,
    ARV_GVCP_ERROR_PACKET_NOT_YET_AVAILABLE =   0x10,
    ARV_GVCP_ERROR_PACKET_AND_PREVIOUS_REMOVED_FROM_MEMORY =    0x11,
    ARV_GVCP_ERROR_PACKET_REMOVED_FROM_MEMORY = 0x12,
    ARV_GVCP_ERROR_NO_REFERENCE_TIME =          0x13,
    ARV_GVCP_ERROR_PACKET_TEMPORARILY_UNAVAILABLE = 0x14,
    ARV_GVCP_ERROR_OVERFLOW =                   0x15,
    ARV_GVCP_ERROR_ACTION_LATE =    0x16,
    ARV_GVCP_ERROR_LEADER_TRAILER_OVERFLOW =    0x17,
    ARV_GVCP_ERROR_GENERIC =                    0xff
} ArvGvcpError;


/**
 * ArvGvcpCmdPacketFlags:
 * @ARV_GVCP_CMD_PACKET_FLAGS_NONE: no flag defined
 * @ARV_GVCP_CMD_PACKET_FLAGS_ACK_REQUIRED: acknowledge required
 * @ARV_GVCP_CMD_PACKET_FLAGS_EXTENDED_IDS: use extended ids
 */

typedef enum {
    ARV_GVCP_CMD_PACKET_FLAGS_NONE =            0x00,
    ARV_GVCP_CMD_PACKET_FLAGS_ACK_REQUIRED =        0x01,
    ARV_GVCP_CMD_PACKET_FLAGS_EXTENDED_IDS =        0x10,
} ArvGvcpCmdPacketFlags;

/**
 * ArvGvcpEventPacketFlags:
 * @ARV_GVCP_EVENT_PACKET_FLAGS_NONE: no flag defined
 * @ARV_GVCP_EVENT_PACKET_FLAGS_64BIT_ID: extended id
 */

typedef enum {
    ARV_GVCP_EVENT_PACKET_FLAGS_NONE =          0x00,
    ARV_GVCP_EVENT_PACKET_FLAGS_64BIT_ID =          0x10,
} ArvGvcpEventPacketFlags;

/**
 * ArvGvcpDiscoveryPacketFlags:
 * @ARV_GVCP_DISCOVERY_PACKET_FLAGS_NONE: no flag defined
 * @ARV_GVCP_DISCOVERY_PACKET_FLAGS_ALLOW_BROADCAST_ACK: allow broadcast acknowledge
 */

typedef enum {
    ARV_GVCP_DISCOVERY_PACKET_FLAGS_NONE =          0x00,
    ARV_GVCP_DISCOVERY_PACKET_FLAGS_ALLOW_BROADCAST_ACK =   0x10,
} ArvGvcpDiscoveryPacketFlags;


/**
 * ArvGvcpCommand:
 * @ARV_GVCP_COMMAND_DISCOVERY_CMD: discovery command
 * @ARV_GVCP_COMMAND_DISCOVERY_ACK: discovery acknowledge
 * @ARV_GVCP_COMMAND_BYE_CMD: goodbye command, for connection termination
 * @ARV_GVCP_COMMAND_BYE_ACK: goodbye acknowledge
 * @ARV_GVCP_COMMAND_PACKET_RESEND_CMD: packet resend request
 * @ARV_GVCP_COMMAND_PACKET_RESEND_ACK: packet resend acknowledge (not used ?)
 * @ARV_GVCP_COMMAND_READ_REGISTER_CMD: read register command
 * @ARV_GVCP_COMMAND_READ_REGISTER_ACK: read register acknowledge
 * @ARV_GVCP_COMMAND_WRITE_REGISTER_CMD: write register command
 * @ARV_GVCP_COMMAND_WRITE_REGISTER_ACK: write register acknowledge
 * @ARV_GVCP_COMMAND_READ_MEMORY_CMD: read memory command
 * @ARV_GVCP_COMMAND_READ_MEMORY_ACK: read memory acknowledge
 * @ARV_GVCP_COMMAND_WRITE_MEMORY_CMD: write memory command
 * @ARV_GVCP_COMMAND_WRITE_MEMORY_ACK: write memory acknowledge
 * @ARV_GVCP_COMMAND_PENDING_ACK: pending command acknowledge
 */

typedef enum {
    ARV_GVCP_COMMAND_DISCOVERY_CMD =    0x0002,
    ARV_GVCP_COMMAND_DISCOVERY_ACK =    0x0003,
    ARV_GVCP_COMMAND_BYE_CMD =      0x0004,
    ARV_GVCP_COMMAND_BYE_ACK =      0x0005,
    ARV_GVCP_COMMAND_PACKET_RESEND_CMD =    0x0040,
    ARV_GVCP_COMMAND_PACKET_RESEND_ACK =    0x0041,
    ARV_GVCP_COMMAND_READ_REGISTER_CMD =    0x0080,
    ARV_GVCP_COMMAND_READ_REGISTER_ACK =    0x0081,
    ARV_GVCP_COMMAND_WRITE_REGISTER_CMD =   0x0082,
    ARV_GVCP_COMMAND_WRITE_REGISTER_ACK =   0x0083,
    ARV_GVCP_COMMAND_READ_MEMORY_CMD =  0x0084,
    ARV_GVCP_COMMAND_READ_MEMORY_ACK =  0x0085,
    ARV_GVCP_COMMAND_WRITE_MEMORY_CMD = 0x0086,
    ARV_GVCP_COMMAND_WRITE_MEMORY_ACK = 0x0087,
    ARV_GVCP_COMMAND_PENDING_ACK =      0x0089
} PvcGvcpCommand;


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
} __attribute__((packed)) PvcGvcpHeader;

/**
 * ArvGvcpPacket:
 * @header: packet header
 * @data: variable size byte array
 *
 * GVCP packet structure.
 */

typedef struct {
    PvcGvcpHeader header;
    unsigned char data[];
} __attribute__((packed)) PvcGvcpPacket;


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
  uint16_t gvcp_packet_get_packet_id (PvcGvcpPacket *packet);
  void gvcp_packet_get_write_register_cmd_infos (const PvcGvcpPacket *packet, uint32_t *address, uint32_t *value);
  void gvcp_packet_get_read_register_cmd_infos (const PvcGvcpPacket *packet, uint32_t *address);
  uint32_t gvcp_packet_get_read_register_ack_value (const PvcGvcpPacket *packet);
  bool gev_camera_read_memory (uint32_t address, uint32_t size, void *buffer);

  PvcGvcpPacket * gvcp_packet_new_read_register_ack (uint32_t value, uint16_t packet_id, size_t *packet_size);
  PvcGvcpPacket * gvcp_packet_new_read_register_cmd (uint32_t address, uint16_t packet_id, size_t *packet_size);
  PvcGvcpPacket * gvcp_packet_new_read_memory_cmd (uint32_t address, uint32_t size, uint16_t packet_id, size_t *packet_size);
  PvcGvcpPacket * gvcp_packet_new_discovery_cmd (bool allow_broadcat_discovery_ack, size_t *packet_size);
  PvcGvcpPacket * gvcp_packet_new_write_register_cmd (uint32_t address, uint32_t value, uint16_t packet_id, size_t *packet_size);

  void waitForReceive();

  void send_complete (const boost::system::error_code& ec, size_t sz);
  void DataReceive (const boost::system::error_code& ec, size_t sz, int num);
  void DataReceive (const boost::system::error_code& ec, size_t sz);

  UDPStreamServer *m_stream_server;

private:
  boost::asio::ip::udp::socket socket;
  boost::asio::ip::udp::endpoint remote_peer;
  char buffer[1536];
  char g_camera_memory[1024];

  //volatile std::sig_atomic_t gSignalStatus;
  int cmd_count;
};


#endif /* __GVCP_HEADER_H__ */

