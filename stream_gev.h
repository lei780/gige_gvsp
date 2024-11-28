
#ifndef __STREAN_GEV_H__
#define __STREAN_GEV_H__

#include <iostream>
#include <thread>
#include <functional>

#include <csignal>
#include <cstdio>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>



#define ARV_FAKE_CAMERA_MEMORY_SIZE 0x10000

#define ARV_GVSP_PACKET_EXTENDED_ID_MODE_MASK   0x80
#define ARV_GVSP_PACKET_ID_MASK         0x00ffffff
#define ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_MASK 0x7f000000
#define ARV_GVSP_PACKET_INFOS_CONTENT_TYPE_POS  24
#define ARV_GVSP_PACKET_INFOS_N_PARTS_MASK      0x000000ff

#define MAXBUF = 1536


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
} __attribute__((packed)) PVC_GvspPacket;


typedef struct {
    uint16_t flags;
    uint32_t packet_infos;
    uint64_t frame_id;
    uint32_t packet_id;
    uint8_t data[];
} __attribute__((packed)) PVC_GvspExtendedHeader;

typedef struct {
    uint32_t pixel_format;
    uint32_t width;
    uint32_t height;
    uint32_t x_offset;
    uint32_t y_offset;
    uint16_t x_padding;
    uint16_t y_padding;
} __attribute__((packed)) PVC_GvspImageInfos;


typedef struct {
    uint16_t flags;
    uint16_t payload_type;
    uint32_t timestamp_high;
    uint32_t timestamp_low;
    ArvGvspImageInfos infos;
} __attribute__((packed)) PVC_GvspImageLeader;


typedef struct {
    uint32_t payload_type;
    uint32_t data0;
} __attribute__((packed)) PVC_GvspTrailer;


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
  UDPAsyncServer(asio::io_service& service, unsigned short port);
  ~UDPAsyncServer();

  void waitForReceive();
  void send_complete (const sys::error_code& ec, size_t sz);
  void DataReceive (const sys::error_code& ec, size_t sz)

  int initDevice(const char *dev_name);
  int queryFormat();

  int startDevice();
  int stopDevice();

private:
  int m_fd;

  asio::ip::udp::socket socket;
  asio::ip::udp::endpoint remote_peer;
  char buffer[MAXBUF];

  volatile std::sig_atomic_t gSignalStatus;
  boost::atomic<bool> init_done (false);

};


#endif /*  __STREAN_GEV_H__ */
