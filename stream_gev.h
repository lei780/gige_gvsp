
#ifndef __STREAN_GEV_H__
#define __STREAN_GEV_H__

#include <iostream>
#include <thread>
#include <functional>

#include <csignal>
#include <cstdio>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define ARV_FAKE_CAMERA_MEMORY_SIZE 0x10000

#define ARV_GVSP_PACKET_EXTENDED_ID_MODE_MASK   0x80
#define ARV_GVSP_PACKET_ID_MASK         0x00ffffff
#define ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_MASK 0x7f000000
#define ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_POS  24
#define ARV_GVSP_PACKET_INFOS_N_PARTS_MASK      0x000000ff

#define MAX_FRAME_BUFFER 1536

/* Minimum ethernet frame size minus ethernet protocol overhead */
#define ARV_GVSP_MINIMUM_PACKET_SIZE           (64 - 14 - 4)

/* Maximum ethernet frame size minus ethernet protocol overhead */
#define ARV_GVSP_MAXIMUM_PACKET_SIZE           (65536 - 14 - 4)

 /* IP + UDP */
#define ARV_GVSP_PACKET_UDP_OVERHEAD           (20 + 8)

 /* IP + UDP + GVSP headers or IP + UDP + GVSP extended headers */
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



typedef enum {
    ARV_GVSP_CONTENT_TYPE_LEADER =      0x01,
    ARV_GVSP_CONTENT_TYPE_TRAILER =     0x02,
    ARV_GVSP_CONTENT_TYPE_PAYLOAD =     0x03,
    ARV_GVSP_CONTENT_TYPE_ALL_IN =      0x04,
    ARV_GVSP_CONTENT_TYPE_H264 =        0x05,
    ARV_GVSP_CONTENT_TYPE_MULTIZONE =   0x06,
    ARV_GVSP_CONTENT_TYPE_MULTIPART =   0x07,
    ARV_GVSP_CONTENT_TYPE_GENDC =       0x08
} PvcGvspContentType;


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
} PvcBufferPayloadType;



typedef struct {
    uint16_t packet_type;
    uint8_t header[];
} __attribute__((packed)) PvcGvspPacket;


typedef struct {
    uint16_t flags;
    uint32_t packet_infos;
    uint64_t frame_id;
    uint32_t packet_id;
    uint8_t data[];
} __attribute__((packed)) PvcGvspExtendedHeader;

typedef struct {
    uint32_t pixel_format;
    uint32_t width;
    uint32_t height;
    uint32_t x_offset;
    uint32_t y_offset;
    uint16_t x_padding;
    uint16_t y_padding;
} __attribute__((packed)) PvcGvspImageInfos;


typedef struct {
    uint16_t flags;
    uint16_t payload_type;
    uint32_t timestamp_high;
    uint32_t timestamp_low;
    PvcGvspImageInfos infos;
} __attribute__((packed)) PvcGvspImageLeader;


typedef struct {
    uint16_t frame_id;
    uint32_t packet_infos;
    uint8_t data[];
} __attribute__((packed)) PvcGvspHeader;


typedef struct {
    uint32_t payload_type;
    uint32_t data0;
} __attribute__((packed)) PvcGvspTrailer;


typedef struct {
    uint16_t flags;
    uint16_t payload_type;
    uint32_t timestamp_high;
    uint32_t timestamp_low;
} __attribute__((packed)) PvcGvspLeader;


typedef struct {
    uint8_t part_id;
    uint8_t zone_info;
    uint16_t offset_high;
    uint32_t offset_low;
} __attribute__((packed)) PvcGvspMultipart;


enum V4l2IoType
{
    IOTYPE_READWRITE,
    IOTYPE_MMAP
};

// ---------------------------------
// V4L2 Device parameters
// ---------------------------------
struct V4L2DeviceParameters
{
    V4L2DeviceParameters( const char* devname, 
                          unsigned int format, 
                          unsigned int width, 
                          unsigned int height, 
                          int fps, V4l2IoType ioType = IOTYPE_MMAP, 
                          int openFlags = O_RDWR | O_NONBLOCK ) :
     m_devName(devname), m_width(width), m_height(height), m_fps(fps), m_iotype(ioType), m_openFlags(openFlags) 
     {

        if (format) {
            m_formatList.push_back(format);
        }
    }

    std::string m_devName;
    std::list<unsigned int> m_formatList;

    unsigned int m_width;
    unsigned int m_height;
    int m_fps;
    V4l2IoType m_iotype;
    int m_verbose;
    int m_openFlags;
};


class UDPStreamServer
{
public:
  UDPStreamServer(boost::asio::io_service& service, unsigned short port)
     : socket(service,
          boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port))
  {
    init_done = false;
    waitForReceive();
    m_transfer_packet_length = 1480;
  }

  virtual ~UDPStreamServer() ;

  void waitForReceive(void);
  void send_complete (const boost::system::error_code& ec, size_t sz);
  void frame_udp_send (char *pdata, unsigned int length) ;
  void DataReceive (const boost::system::error_code& ec, size_t sz) ;

  bool gvsp_packet_has_extended_ids (const PvcGvspPacket *packet);

  void * gvsp_packet_get_data (const PvcGvspPacket *packet);

  PvcGvspPacket * gvsp_packet_new (PvcGvspContentType content_type,
             uint16_t frame_id, uint32_t packet_id, size_t data_size, void *buffer, size_t *buffer_size);

  int PvcInitDevice(const char *dev_name);
  void PvcVideoframePolling(void);

  int PvcStartDevice(void);
  int PvcStopDevice(void);

  //void signal_handler(int signal);
  int video_stream_mode ;

private:
  int m_video_fd = -1;
  //unsigned int m_frame_id = 0;
  //unsigned int m_packet_id = 0;

  boost::asio::ip::udp::socket socket;
  boost::asio::ip::udp::endpoint remote_peer;
  char buffer[1536];

  //volatile std::sig_atomic_t gSignalStatus;

  boost::atomic<bool> init_done;
  boost::atomic<int> loop ;

  std::thread frame_proc ;
  std::mutex m_mode_lock;

  std::queue< std::vector<PvcGvspPacket *> > frames;
  std::vector<char *> bufferof_packets;

  unsigned int m_transfer_packet_length;
  int m_fd;
};

#endif /*  __STREAN_GEV_H__ */

