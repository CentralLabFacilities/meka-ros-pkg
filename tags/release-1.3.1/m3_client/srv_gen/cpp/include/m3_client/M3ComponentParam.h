/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/meka-ros-pkg/m3_client/srv/M3ComponentParam.srv */
#ifndef M3_CLIENT_SERVICE_M3COMPONENTPARAM_H
#define M3_CLIENT_SERVICE_M3COMPONENTPARAM_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace m3_client
{
template <class ContainerAllocator>
struct M3ComponentParamRequest_ : public ros::Message
{
  typedef M3ComponentParamRequest_<ContainerAllocator> Type;

  M3ComponentParamRequest_()
  : a(0)
  {
  }

  M3ComponentParamRequest_(const ContainerAllocator& _alloc)
  : a(0)
  {
  }

  typedef int32_t _a_type;
  int32_t a;


private:
  static const char* __s_getDataType_() { return "m3_client/M3ComponentParamRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "5c9fb1a886e81e3162a5c87bf55c072b"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "25ba3fa9d5d930574c4d72dc4151cd60"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 a\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, a);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, a);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(a);
    return size;
  }

  typedef boost::shared_ptr< ::m3_client::M3ComponentParamRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3_client::M3ComponentParamRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct M3ComponentParamRequest
typedef  ::m3_client::M3ComponentParamRequest_<std::allocator<void> > M3ComponentParamRequest;

typedef boost::shared_ptr< ::m3_client::M3ComponentParamRequest> M3ComponentParamRequestPtr;
typedef boost::shared_ptr< ::m3_client::M3ComponentParamRequest const> M3ComponentParamRequestConstPtr;


template <class ContainerAllocator>
struct M3ComponentParamResponse_ : public ros::Message
{
  typedef M3ComponentParamResponse_<ContainerAllocator> Type;

  M3ComponentParamResponse_()
  : b(0)
  {
  }

  M3ComponentParamResponse_(const ContainerAllocator& _alloc)
  : b(0)
  {
  }

  typedef int32_t _b_type;
  int32_t b;


private:
  static const char* __s_getDataType_() { return "m3_client/M3ComponentParamResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "976c440660ac67ad67b35c9dce4f2065"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "25ba3fa9d5d930574c4d72dc4151cd60"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 b\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, b);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, b);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(b);
    return size;
  }

  typedef boost::shared_ptr< ::m3_client::M3ComponentParamResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3_client::M3ComponentParamResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct M3ComponentParamResponse
typedef  ::m3_client::M3ComponentParamResponse_<std::allocator<void> > M3ComponentParamResponse;

typedef boost::shared_ptr< ::m3_client::M3ComponentParamResponse> M3ComponentParamResponsePtr;
typedef boost::shared_ptr< ::m3_client::M3ComponentParamResponse const> M3ComponentParamResponseConstPtr;

struct M3ComponentParam
{

typedef M3ComponentParamRequest Request;
typedef M3ComponentParamResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct M3ComponentParam
} // namespace m3_client

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3_client::M3ComponentParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5c9fb1a886e81e3162a5c87bf55c072b";
  }

  static const char* value(const  ::m3_client::M3ComponentParamRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5c9fb1a886e81e31ULL;
  static const uint64_t static_value2 = 0x62a5c87bf55c072bULL;
};

template<class ContainerAllocator>
struct DataType< ::m3_client::M3ComponentParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3ComponentParamRequest";
  }

  static const char* value(const  ::m3_client::M3ComponentParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3_client::M3ComponentParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 a\n\
\n\
";
  }

  static const char* value(const  ::m3_client::M3ComponentParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3_client::M3ComponentParamRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3_client::M3ComponentParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "976c440660ac67ad67b35c9dce4f2065";
  }

  static const char* value(const  ::m3_client::M3ComponentParamResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x976c440660ac67adULL;
  static const uint64_t static_value2 = 0x67b35c9dce4f2065ULL;
};

template<class ContainerAllocator>
struct DataType< ::m3_client::M3ComponentParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3ComponentParamResponse";
  }

  static const char* value(const  ::m3_client::M3ComponentParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3_client::M3ComponentParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 b\n\
\n\
";
  }

  static const char* value(const  ::m3_client::M3ComponentParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3_client::M3ComponentParamResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3_client::M3ComponentParamRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.a);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3ComponentParamRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3_client::M3ComponentParamResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.b);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3ComponentParamResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<m3_client::M3ComponentParam> {
  static const char* value() 
  {
    return "25ba3fa9d5d930574c4d72dc4151cd60";
  }

  static const char* value(const m3_client::M3ComponentParam&) { return value(); } 
};

template<>
struct DataType<m3_client::M3ComponentParam> {
  static const char* value() 
  {
    return "m3_client/M3ComponentParam";
  }

  static const char* value(const m3_client::M3ComponentParam&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3_client::M3ComponentParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "25ba3fa9d5d930574c4d72dc4151cd60";
  }

  static const char* value(const m3_client::M3ComponentParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3_client::M3ComponentParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3ComponentParam";
  }

  static const char* value(const m3_client::M3ComponentParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3_client::M3ComponentParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "25ba3fa9d5d930574c4d72dc4151cd60";
  }

  static const char* value(const m3_client::M3ComponentParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3_client::M3ComponentParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3ComponentParam";
  }

  static const char* value(const m3_client::M3ComponentParamResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // M3_CLIENT_SERVICE_M3COMPONENTPARAM_H

