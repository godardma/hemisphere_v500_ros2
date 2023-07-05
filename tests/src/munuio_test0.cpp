#include <iostream>
using namespace std;

#include <munu_io/AsyncService.h>
#include <munu_io/SerialDevice.h>
#include <munu_io/ClientTCP.h>
using namespace munu;

void serial_callback(SerialDevice<>* serial,
                     const boost::system::error_code& err,
                     const std::string& data)
{
    //cout << data << endl;
    serial->async_read_until('\n', boost::bind(serial_callback, serial, _1, _2));
}

void tcp_callback(ClientTCP<>* tcp, SerialDevice<>* serial,
                  std::vector<char>* buf,
                  const boost::system::error_code& err,
                  size_t byteCount)

{
    if(err) {
        std::cout << "Correction reception error" << std::endl;
        return;
    }
    serial->write(buf->size(), buf->data()); // this is synchronous
    tcp->async_read(buf->size(), buf->data(),
                    boost::bind(tcp_callback, tcp, serial, buf, _1, _2));
}

int main()
{
    AsyncService service;
    SerialDevice<> serial(*service.io_service());
    serial.open("/dev/narval_usbl", 115200);
    serial.async_read_until('\n', boost::bind(serial_callback, &serial, _1, _2));

    std::vector<char> buffer(32);
    ClientTCP<> tcp(*service.io_service());
    tcp.open("127.0.0.1", 28334);
    tcp.async_read(buffer.size(), buffer.data(),
                   boost::bind(tcp_callback, &tcp, &serial, &buffer, _1, _2));

    service.start();

    getchar();

    service.stop();

    return 0;
}
