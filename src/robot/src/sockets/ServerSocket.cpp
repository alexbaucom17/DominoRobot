// Implementation of the ServerSocket class

#include "ServerSocket.h"
#include "SocketException.h"
#include "SocketTimeoutException.h"
#include <iostream>


ServerSocket::ServerSocket ( int port )
{
  if ( ! Socket::create() )
    {
      throw SocketException ( "Could not create server socket." );
    }

  if ( ! Socket::bind ( port ) )
    {
      throw SocketException ( "Could not bind to port." );
    }

  if ( ! Socket::listen() )
    {
      throw SocketException ( "Could not listen to socket." );
    }

}

ServerSocket::~ServerSocket()
{
}


const ServerSocket& ServerSocket::operator << ( const std::string& s ) const
{
  if ( ! Socket::send ( s ) )
    {
      throw SocketException ( "Could not write to socket." );
    }

  return *this;

}


const ServerSocket& ServerSocket::operator >> ( std::string& s ) const
{
  int status = Socket::recv ( s );
  if ( status == -1 )
  {
    throw SocketTimeoutException ( "No data available in blocking time" );
  }
  else if (status == 0)
  {
    throw SocketException ( "Could not read from socket." );
  }

  return *this;
}

void ServerSocket::accept ( ServerSocket& sock )
{
  if ( ! Socket::accept ( sock ) )
    {
      throw SocketException ( "Could not accept socket." );
    }
}

void ServerSocket::set_non_blocking()
{
  Socket::set_non_blocking(true);
}