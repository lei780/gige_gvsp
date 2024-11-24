#include <boost/asio.hpp>
#include <memory>
#include <boost/thread.hpp>
#include <iostream>
namespace asio = boost::asio;

typedef std::unique_ptr<asio::io_service::work> work_ptr;

#define PRINT_ARGS(msg) do {\
  boost::lock_guard<boost::mutex> lg(mtx); \
  std::cout << '[' << boost::this_thread::get_id() \
            << "] " << msg << std::endl; \
} while (0)


class UDPClient
{
public:
    boost::asio::io_service io_service;

    udp::socket* socket;
    udp::endpoint* receiver_endpoint;
    boost::array<char, 1024> recv_buffer;

    UDPClient();
    void do_receive();
    void handle_receive(const boost::system::error_code& error, size_t);
};

#if 1
UDPClient::UDPClient()
{
    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), "127.0.0.1", "8888");
    receiver_endpoint = new udp::endpoint(*resolver.resolve(query));

	//udp::socket socket(io_service, udp::endpoint(boost::asio::ip::udp::v4(), CLIENT_PORT_NUMBER));
	//auto endpoint = udp::endpoint( boost::asio::ip::make_address(UDP_IP), SERVER_PORT_NUMBER);

    socket = new udp::socket(io_service);
    socket->open(udp::v4());

    do_receive();
    io_service.run();

    //while (true)
    //{
    //    io_service.poll();
    //    Sleep(1);
    //}
}
#endif

#if 0
UDPClient::UDPClient()
    : io_service(),
      socket(io_service, {udp::v4(), 8888})
{
    do_receive();
    io_service.run();
}
#endif


void UDPClient::do_receive()
{
    socket->async_receive_from(boost::asio::buffer(recv_buffer), *receiver_endpoint,
                                 boost::bind(&UDPClient::handle_receive, this,
                                 boost::asio::placeholders::error,
                                 boost::asio::placeholders::bytes_transferred));

    //boost::asio::async_read(_socket, mutableBuffer, boost::asio::transfer_at_least(1), boost::bind(&connection::read_handler, this, _1, _2));

}

void UDPClient::handle_receive(const boost::system::error_code& error, size_t bytes_transferred)
{
    cout << "ulala" << endl;

    if (!error || error == boost::asio::error::message_size)
        do_receive();
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
      //const char *msg = "hello from server";
      ArvGvcpPacket *gvcp_packet = (ArvGvcpPacket *)buffer;

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


int main() {
#if 0
  asio::io_service service;
  // keep the workers occupied
  work_ptr work(new asio::io_service::work(service));
  boost::mutex mtx;

  // set up the worker threads in a thread group
  boost::thread_group workers;
  for (int i = 0; i < 3; ++i) {
    workers.create_thread([&service, &mtx]() {
                         PRINT_ARGS("Starting worker thread ");
                         service.run();
                         PRINT_ARGS("Worker thread done");
                       });
  }

  // Post work
  for (int i = 0; i < 20; ++i) {
    service.post(
      [&service, &mtx]() {
        PRINT_ARGS("Hello, world!");
        service.post([&mtx]() {
                           PRINT_ARGS("Hola, mundo!");
                         });
      });
  }

  work.reset(); // destroy work object: signals end of work
  workers.join_all(); // wait for all worker threads to finish
#endif

  UDPClient updclient;

}

