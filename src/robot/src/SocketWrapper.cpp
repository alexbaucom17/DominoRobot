#include "SocketWrapper.h"


SocketWrapper::SocketWrapper()
: socket(kn::endpoint("0.0.0.0", port))
{
    socket.bind();
    socket.listen();
}

void SocketWrapper::run()
{

}

std::string SocketWrapper::getData()
{
    return "Hi";
}

void SocketWrapper::sendData(std::string data)
{

}

void SocketWrapper::socket_loop()
{
    while(true)
    {
        // Wait for a client to connect
        auto client = socket.accept();
        bool keep_connection = true;

        // Stay connected to the client while the connection is valid
        while(keep_connection)
        {
            // Try to read data from the client
            kn::buffer<1024> read_buff;
            const auto [size, status] = client.recv(buff);

            // If read fails, disconnect
            if(!status)
            {
                keep_connection = false;
            }
            else
            {
                // If client has disconnected, disconnect
                if(status.value == kn::socket_status::cleanly_disconnected)
                {
                    continue_receiving = false;
                }
                else
                {
                    // Copy data into buffer for output if we got info from the client
                    const std::lock_guard<std::mutex> lock(read_mutex);
                    for (int i = 0; i < size; i++)
                    {
                        data_buffer.append(read_buff[i]);
                    }

                    // If we have data ready to be sent, send it 
                    if(lentgh_to_send > 0)
                    {
                        client.send(send_buffer.data, lentgh_to_send);
                    }
                }
            }

        }

    }

}
