// SocketTimeoutException class


#ifndef SocketTimeoutException_class
#define SocketTimeoutException_class

#include <string>

class SocketTimeoutException
{
 public:
  SocketTimeoutException ( std::string s ) : m_s ( s ) {};
  ~SocketTimeoutException (){};

  std::string description() { return m_s; }

 private:

  std::string m_s;

};

#endif