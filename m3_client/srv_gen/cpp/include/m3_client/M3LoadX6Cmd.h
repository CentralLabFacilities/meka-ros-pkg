/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/meka-ros-pkg/m3_client/srv/M3LoadX6Cmd.srv */
#ifndef M3_CLIENT_SERVICE_M3LOADX6CMD_H
#define M3_CLIENT_SERVICE_M3LOADX6CMD_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace m3_client
{
template <class ContainerAllocator>
struct M3LoadX6CmdRequest_ {
  typedef M3LoadX6CmdRequest_<ContainerAllocator> Type;

  M3LoadX6CmdRequest_()
  : request(0)
  {
  }

  M3LoadX6CmdRequest_(const ContainerAllocator& _alloc)
  : request(0)
  {
  }

  typedef int32_t _request_type;
  int32_t request;


  typedef boost::shared_ptr< ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct M3LoadX6CmdRequest
typedef  ::m3_client::M3LoadX6CmdRequest_<std::allocator<void> > M3LoadX6CmdRequest;

typedef boost::shared_ptr< ::m3_client::M3LoadX6CmdRequest> M3LoadX6CmdRequestPtr;
typedef boost::shared_ptr< ::m3_client::M3LoadX6CmdRequest const> M3LoadX6CmdRequestConstPtr;


template <class ContainerAllocator>
struct M3LoadX6CmdResponse_ {
  typedef M3LoadX6CmdResponse_<ContainerAllocator> Type;

  M3LoadX6CmdResponse_()
  : response(0)
  {
  }

  M3LoadX6CmdResponse_(const ContainerAllocator& _alloc)
  : response(0)
  {
  }

  typedef int32_t _response_type;
  int32_t response;


  typedef boost::shared_ptr< ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct M3LoadX6CmdResponse
typedef  ::m3_client::M3LoadX6CmdResponse_<std::allocator<void> > M3LoadX6CmdResponse;

typedef boost::shared_ptr< ::m3_client::M3LoadX6CmdResponse> M3LoadX6CmdResponsePtr;
typedef boost::shared_ptr< ::m3_client::M3LoadX6CmdResponse const> M3LoadX6CmdResponseConstPtr;

struct M3LoadX6Cmd
{

typedef M3LoadX6CmdRequest Request;
typedef M3LoadX6CmdResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct M3LoadX6Cmd
} // namespace m3_client

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "650f0ccd41c8f8d53ada80be6ddde434";
  }

  static const char* value(const  ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x650f0ccd41c8f8d5ULL;
  static const uint64_t static_value2 = 0x3ada80be6ddde434ULL;
};

template<class ContainerAllocator>
struct DataType< ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3LoadX6CmdRequest";
  }

  static const char* value(const  ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 request\n\
\n\
";
  }

  static const char* value(const  ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f45f68e2feefb1307598e828e260b7d7";
  }

  static const char* value(const  ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf45f68e2feefb130ULL;
  static const uint64_t static_value2 = 0x7598e828e260b7d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3LoadX6CmdResponse";
  }

  static const char* value(const  ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 response\n\
\n\
";
  }

  static const char* value(const  ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3_client::M3LoadX6CmdRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.request);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3LoadX6CmdRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3_client::M3LoadX6CmdResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.response);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3LoadX6CmdResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<m3_client::M3LoadX6Cmd> {
  static const char* value() 
  {
    return "51edd9dfd50014fde2b589cbf77706aa";
  }

  static const char* value(const m3_client::M3LoadX6Cmd&) { return value(); } 
};

template<>
struct DataType<m3_client::M3LoadX6Cmd> {
  static const char* value() 
  {
    return "m3_client/M3LoadX6Cmd";
  }

  static const char* value(const m3_client::M3LoadX6Cmd&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3_client::M3LoadX6CmdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "51edd9dfd50014fde2b589cbf77706aa";
  }

  static const char* value(const m3_client::M3LoadX6CmdRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3_client::M3LoadX6CmdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3LoadX6Cmd";
  }

  static const char* value(const m3_client::M3LoadX6CmdRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3_client::M3LoadX6CmdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "51edd9dfd50014fde2b589cbf77706aa";
  }

  static const char* value(const m3_client::M3LoadX6CmdResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3_client::M3LoadX6CmdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3LoadX6Cmd";
  }

  static const char* value(const m3_client::M3LoadX6CmdResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // M3_CLIENT_SERVICE_M3LOADX6CMD_H
