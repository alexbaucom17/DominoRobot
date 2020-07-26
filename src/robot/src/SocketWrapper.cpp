#include "SocketWrapper.h"

#include <plog/Log.h>
#include <mutex>

std::mutex read_mutex;
std::mutex send_mutex;

SocketWrapper::SocketWrapper()
: length_to_send(0)
{
    run_thread = std::thread(&SocketWrapper::socket_loop, this);
    run_thread.detach();
}

bool SocketWrapper::dataAvailableToRead()
{
    std::lock_guard<std::mutex> read_lock(read_mutex);
    return data_buffer.size() > 0;
}

std::string SocketWrapper::getData()
{
    std::string outstring;
    const std::lock_guard<std::mutex> lock(read_mutex);
    while(!data_buffer.empty())
    {
        outstring.push_back(static_cast<char>(data_buffer.front()));
        data_buffer.pop();
    }
    return outstring;
}

void SocketWrapper::sendData(std::string data)
{
    const std::lock_guard<std::mutex> lock(send_mutex);
    if(length_to_send + data.size() >= BUFFER_SIZE)
    {
        PLOGE.printf("Send buffer overflow, dropping message: %s", data.c_str());
        return;
    }
    for(const char c : data)
    {
        send_buffer[length_to_send] = static_cast<std::byte>(c);
        length_to_send++;
    }
    PLOGI.printf("Length to send: %i", length_to_send);
}

void SocketWrapper::socket_loop()
{
    kn::socket<kissnet::protocol::tcp> socket(kn::endpoint("0.0.0.0:8123"));
    socket.bind();
    socket.listen();

    while(true)
    {
        // Wait for a client to connect
        PLOGI.printf("Ready for client connection");
        auto client = socket.accept();
        bool keep_connection = true;
        PLOGI.printf("Client connected");

        // Stay connected to the client while the connection is valid
        while(keep_connection)
        {
            // Try to read data from the client
            kn::buffer<BUFFER_SIZE> read_buff;
            const auto [read_size, status] = client.recv(read_buff);

            PLOGD.printf("Read size: %i", read_size);

            // If read fails, disconnect
            if(!status)
            {
                keep_connection = false;
                PLOGE.printf("Read error");
            }
            else
            {
                // If client has disconnected, disconnect
                if(status.value == kn::socket_status::cleanly_disconnected)
                {
                    keep_connection = false;
                    PLOGI.printf("Client disconnected cleanly");
                }
                else
                {
                    {
                        // Copy data into buffer for output if we got info from the client
                        std::lock_guard<std::mutex> read_lock(read_mutex);
                        if(data_buffer.size() + read_size >= BUFFER_SIZE)
                        {
                            PLOGE.printf("Data buffer overflow, dropping message");
                        }
                        else
                        {
                            for (uint i = 0; i < read_size; i++)
                            {
                                //PLOGD << std::hex << std::to_integer<int>(read_buff[i]) << std::dec << ' ';
                                data_buffer.push(read_buff[i]);
                            }
                        }
                    }

                    {
                        // If we have data ready to be sent, send it
                        std::lock_guard<std::mutex> send_lock(send_mutex);
                        PLOGI.printf("Length to send: %i", length_to_send);
                        if(length_to_send > 0)
                        {
                            client.send(send_buffer.data(), length_to_send);
                            length_to_send = 0;
                        }
                    }
                }
            }
        }

        PLOGI.printf("Closing socket connection");
        length_to_send = 0;

    }
}
