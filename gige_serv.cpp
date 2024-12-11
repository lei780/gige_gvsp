
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

#include "stream_gev.h"
#include "gev_gvcp.h"

namespace asio = boost::asio;
namespace sys = boost::system;
using namespace boost::placeholders;

const size_t MAXBUF = 256;
char g_camera_memory[1024];

UDPStreamServer *server = nullptr;
UDPAsyncCMDServer *command_server = nullptr;

asio::io_service service;
asio::io_service command_service;

void signal_handler(int signal)
{
    std::cout << "Signal exec.. " << '\n';

    if(server != nullptr)
        delete server;

    if(command_server != nullptr)
        delete command_server;
}

void stream_proc(void *pdat)
{
    asio::io_service *srv = (asio::io_service *)pdat;
    srv->run();
}

void command_proc(void *pdat)
{
    asio::io_service *srv = (asio::io_service *)pdat;
    srv->run();
}

int main() 
{
  //server = new UDPAsyncServer(service, 55000);
  //server = new UDPAsyncServer(service, 8108);
  server = new UDPStreamServer(service, 8108);

  command_server = new UDPAsyncCMDServer(command_service, ARV_GVCP_PORT);
  command_server->m_stream_server = server;

  server->PvcInitDevice("/dev/video0");

  std::signal(SIGINT, signal_handler);

  //service.run();
  std::function<void()> func = std::bind(&stream_proc, &service);
  std::thread my_thread(func);

  std::function<void()> func1 = std::bind(&command_proc, &command_service);
  std::thread my_thread1(func1);

  server->video_stream_mode = 0;
  server->PvcStartDevice();

  std::cout << "service run.. " << '\n';

  my_thread.join();
  my_thread1.join();
}


