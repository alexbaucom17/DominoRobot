#include "SocketWrapper.h"
#include <plog/Log.h>


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
        PLOGE.printf("Send buffer overflow, dropping message: %s", data);
        return;
    }
    for(const char c : data)
    {
        send_buffer[length_to_send] = static_cast<std::byte>(c);
        length_to_send++;
    }
}

void SocketWrapper::socket_loop()
{
    while(true)
    {
        // Wait for a client to connect
        auto client = socket.accept();
        bool keep_connection = true;
        PLOGI.printf("Client connected");

        // Stay connected to the client while the connection is valid
        while(keep_connection)
        {
            // Try to read data from the client
            kn::buffer<BUFFER_SIZE> read_buff;
            const auto [read_size, status] = client.recv(read_buff);

            // If read fails, disconnect
            if(!status)
            {
                keep_connection = false;
                PLOGI.printf("Read error");
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
                    // Copy data into buffer for output if we got info from the client
                    const std::lock_guard<std::mutex> read_lock(read_mutex);
                    if(data_buffer.size() + read_size >= BUFFER_SIZE)
                    {
                        PLOGE.printf("Data buffer overflow, dropping message");
                    }
                    else
                    {
                        for (uint i = 0; i < read_size; i++)
                        {
                            data_buffer.push(read_buff[i]);
                        }
                    }

                    // If we have data ready to be sent, send it
                    const std::lock_guard<std::mutex> send_lock(send_mutex);
                    if(length_to_send > 0)
                    {
                        client.send(send_buffer.data(), length_to_send);
                        length_to_send = 0;
                    }
                }
            }
        }

        PLOGI.printf("Closing socket connection");

    }
}
