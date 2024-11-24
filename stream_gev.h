
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


typedef struct {
    uint16_t packet_type;
    uint8_t header[];
} __attribute__((packed)) ArvGvspPacket;


typedef struct {
    uint16_t flags;
    uint32_t packet_infos;
    uint64_t frame_id;
    uint32_t packet_id;
    uint8_t data[];
} __attribute__((packed)) ArvGvspExtendedHeader;


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
