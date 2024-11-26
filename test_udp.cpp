
#include <iostream>
#include <vector>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost::asio;
void asyncReadHandler( const boost::system::error_code& error, std::size_t bytesTransferred );
void timeoutHandler( const boost::system::error_code& error, bool* ptime_out );

size_t ReceivedDataSize;
std::string ReadError;

int main(int argc, char * argv[])
{
    io_service io;
    ip::udp::socket socket(io, ip::udp::endpoint(ip::udp::v4(), 1620));
    size_t num = 0;
    while (true)
    {
        std::vector<unsigned char> vec(1500);
        ip::udp::endpoint from;
        socket.async_receive_from( boost::asio::buffer( vec ), from,
                                   boost::bind(
                                       asyncReadHandler,
                                       boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred ) );
        bool timeout = false;
        ReceivedDataSize = 0;
        ReadError = "";

        // Creating and starting timer (by registering timeout handler)
        deadline_timer timer( io, boost::posix_time::seconds( 1 ) );
        timer.async_wait(
            boost::bind( timeoutHandler, boost::asio::placeholders::error, &timeout ) );

        // Resetting IO service instance
        io.reset();

        while(io.run_one())
        {
            if ( timeout ) {
                socket.cancel();
                timer.cancel();
                //Leave the io run_one loop
                break;
            }
            else if ( (0 != ReceivedDataSize ) || (!ReadError.empty())) {
                timer.cancel();
                socket.cancel();
                std::cout << "Received n°" <<  num++ << ": " << ReceivedDataSize << "\r" << std::flush;

                if (0 != ReceivedDataSize )
                    vec.resize(ReceivedDataSize);

                if (!ReadError.empty())
                    std::cout << "Error: " << ReadError << std::endl;

                bool result = true;
                for ( auto x : vec )
                    if ( 0xA5 != x ) { result = false; break; }

                if ( false == result ) {
                    std::cout << std::endl << "Bad reception" << std::endl << std::hex;
                    for ( auto x : vec )
                        std::cout << (int)x << " ";

                    std::cout << std::dec << "\n";
                }
                //Leave the io run_one loop
                break;
            }
            else {
                //What shall I do here ???
                //another potential io.reset () did not bring much
            }

        }
    }

    return 0;
}

void asyncReadHandler( const boost::system::error_code& error, std::size_t bytesTransferred )
{
    // If read canceled, simply returning...
    if( error == boost::asio::error::operation_aborted ) return;

    ReceivedDataSize = 0;

    // If no error
    if( !error ) {
        ReceivedDataSize = bytesTransferred;
    }
    else {
        ReadError = error.message();
    }
}

void timeoutHandler( const boost::system::error_code& error, bool* ptime_out )
{
    // If timer canceled, simply returning...
    if( error == boost::asio::error::operation_aborted ) return;

    // Setting timeout flag
    *ptime_out = true;
}


