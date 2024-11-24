


class UDPAsyncServer
{

public:
  UDPAsyncServer(asio::io_service& service, unsigned short port)
     : socket(service,
          asio::ip::udp::endpoint(asio::ip::udp::v4(), port))
  {  waitForReceive();  }

  ~UDPAsyncServer()
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
           boost::bind(&UDPAsyncServer::DataReceive, this,
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
      const char *msg = "hello from server";

      if( ec == boost::asio::error::operation_aborted )
      {
          std::cout << __FUNCTION__ << ": Aborted "  << '\n';
          return;
      }

      waitForReceive();

      std::cout << "Received: [" << buffer << "] " << remote_peer << '\n';

      socket.async_send_to(
          asio::buffer(msg, strlen(msg)),
          remote_peer,
          boost::bind(&UDPAsyncServer::send_complete, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred) );
  }

#if 0
  void signal_handler(int signal)
  {
      gSignalStatus = signal;
      socket.cancel();
  }
#endif

private:
  asio::ip::udp::socket socket;
  asio::ip::udp::endpoint remote_peer;
  char buffer[MAXBUF];

  volatile std::sig_atomic_t gSignalStatus;
  boost::atomic<bool> done (false);
};

