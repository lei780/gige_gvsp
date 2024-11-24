
class stdin_stream: boost::noncopyable 
{
	typedef boost::function<void (const boost::system::error_code &, size_t bytes_transferred)> read_handler_type;

public:
	stdin_stream(boost::asio::io_service &io, HANDLE hin): io(io), hin(hin), hev(CreateEvent(0, 0, 0, 0)), handler(), buffer(0), size(0) {
		_beginthread(&stdin_stream::thread_handler_gateway, 0, this);
	}

	~stdin_stream() {
		buffer = 0;
		CloseHandle(hev);
	}

	void async_read_some(const boost::asio::mutable_buffer &seq, read_handler_type handler) {
		if ((size = boost::asio::buffer_size(seq)) == 0) {
			io.dispatch(boost::bind(handler, boost::system::error_code(), 0));
			return;
		}
		buffer = boost::asio::buffer_cast<char *>(seq);
		this->handler = handler;
		SetEvent(hev);
	}

	void thread_handler() {
		while (WaitForSingleObject(hev, INFINITE)==0) { // worked in another thread
			DWORD err = 0;
			if (!ReadFile(hin, buffer, size, const_cast<DWORD *>(&size), 0)) err = GetLastError();
			io.dispatch(boost::bind(handler, boost::system::error_code(err, boost::system::system_category()), size));
		}
	}

	static void thread_handler_gateway(void *this_p) { static_cast<stdin_stream *>(this_p)->thread_handler(); }
	boost::asio::io_service &io_service() { return io; }

private:
	boost::asio::io_service &io;
	HANDLE hin;
	HANDLE hev;
	read_handler_type handler;
	char *volatile buffer;
	volatile DWORD size;
};

