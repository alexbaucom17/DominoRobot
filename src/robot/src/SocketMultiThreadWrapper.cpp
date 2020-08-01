#include "SocketMultiThreadWrapper.h"

#include <plog/Log.h>
#include "sockets/ServerSocket.h"
#include "sockets/SocketException.h"
#include "sockets/SocketTimeoutException.h"
#include <mutex>

std::mutex read_mutex;
std::mutex send_mutex;

SocketMultiThreadWrapper::SocketMultiThreadWrapper()
: data_buffer(),
  send_buffer()
{
    run_thread = std::thread(&SocketMultiThreadWrapper::socket_loop, this);
    run_thread.detach();
}

bool SocketMultiThreadWrapper::dataAvailableToRead()
{
    std::lock_guard<std::mutex> read_lock(read_mutex);
    return data_buffer.size() > 0;
}

std::string SocketMultiThreadWrapper::getData()
{
    std::string outstring;
    const std::lock_guard<std::mutex> lock(read_mutex);
    while(!data_buffer.empty())
    {
        outstring.push_back(data_buffer.front());
        data_buffer.pop();
    }
    return outstring;
}

void SocketMultiThreadWrapper::sendData(std::string data)
{
    const std::lock_guard<std::mutex> lock(send_mutex);
    if(send_buffer.size() + data.size() >= BUFFER_SIZE)
    {
        PLOGE.printf("Send buffer overflow, dropping message: %s", data.c_str());
        return;
    }
    for(const char c : data)
    {
        send_buffer.push(c);
    }
}

void SocketMultiThreadWrapper::socket_loop()
{
    ServerSocket server(8123);

    while(true)
    {
        // Wait for a client to connect
        PLOGI.printf("Ready for client connection");
        ServerSocket socket;
        server.accept(socket);
        socket.set_non_blocking();
        bool keep_connection = true;
        PLOGI.printf("Client connected");

        // Stay connected to the client while the connection is valid
        while(keep_connection)
        {
            // Try to read data from the client
            std::string read_data;
            try
            {
                socket >> read_data;
            }
            catch (SocketTimeoutException &e) {}
            catch (SocketException& e)
            {
                //PLOGI.printf("Caught exception while reading: %s", e.description().c_str());
                // If read fails for something other than timeout, disconnect
                keep_connection = false;
                continue;
            }

            {
                // Copy data into buffer for output if we got info from the client
                std::lock_guard<std::mutex> read_lock(read_mutex);
                if(data_buffer.size() + read_data.size() >= BUFFER_SIZE)
                {
                    PLOGE.printf("Data buffer overflow, dropping message");
                }
                else
                {
                    for (uint i = 0; i < read_data.size(); i++)
                    {
                        //PLOGD << std::hex << std::to_integer<int>(read_buff[i]) << std::dec << ' ';
                        data_buffer.push(read_data[i]);
                    }
                }
            }

            {
                // If we have data ready to be sent, send it
                std::lock_guard<std::mutex> send_lock(send_mutex);
                if(send_buffer.size() > 0)
                {
                    PLOGD.printf("Length to send: %i", send_buffer.size());
                    std::string send_data;
                    while(!send_buffer.empty())
                    {
                        send_data.push_back(send_buffer.front());
                        send_buffer.pop();
                    }
                    socket << send_data;
                }

            }
        }

        PLOGI.printf("Closing socket connection");

    }
}
