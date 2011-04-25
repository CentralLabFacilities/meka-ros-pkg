/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/meka-ros-pkg/m3_client/srv/M3HumanoidParam.srv */
#ifndef M3_CLIENT_SERVICE_M3HUMANOIDPARAM_H
#define M3_CLIENT_SERVICE_M3HUMANOIDPARAM_H
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
struct M3HumanoidParamRequest_ : public ros::Message
{
  typedef M3HumanoidParamRequest_<ContainerAllocator> Type;

  M3HumanoidParamRequest_()
  : chain(0)
  , payload_mass(0.0)
  , payload_com()
  , payload_inertia()
  , use_velocities(false)
  , use_accelerations(false)
  {
    payload_com.assign(0.0);
    payload_inertia.assign(0.0);
  }

  M3HumanoidParamRequest_(const ContainerAllocator& _alloc)
  : chain(0)
  , payload_mass(0.0)
  , payload_com()
  , payload_inertia()
  , use_velocities(false)
  , use_accelerations(false)
  {
    payload_com.assign(0.0);
    payload_inertia.assign(0.0);
  }

  typedef uint8_t _chain_type;
  uint8_t chain;

  typedef float _payload_mass_type;
  float payload_mass;

  typedef boost::array<float, 3>  _payload_com_type;
  boost::array<float, 3>  payload_com;

  typedef boost::array<float, 6>  _payload_inertia_type;
  boost::array<float, 6>  payload_inertia;

  typedef uint8_t _use_velocities_type;
  uint8_t use_velocities;

  typedef uint8_t _use_accelerations_type;
  uint8_t use_accelerations;


  ROSCPP_DEPRECATED uint32_t get_payload_com_size() const { return (uint32_t)payload_com.size(); }
  ROSCPP_DEPRECATED uint32_t get_payload_inertia_size() const { return (uint32_t)payload_inertia.size(); }
private:
  static const char* __s_getDataType_() { return "m3_client/M3HumanoidParamRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "1bb1c03ca86da994fbea726ff9330f25"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "34cb417585df77f14f1029d4fc16441a"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "uint8 chain\n\
float32 payload_mass\n\
float32[3] payload_com\n\
float32[6] payload_inertia\n\
bool use_velocities\n\
bool use_accelerations\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, chain);
    ros::serialization::serialize(stream, payload_mass);
    ros::serialization::serialize(stream, payload_com);
    ros::serialization::serialize(stream, payload_inertia);
    ros::serialization::serialize(stream, use_velocities);
    ros::serialization::serialize(stream, use_accelerations);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, chain);
    ros::serialization::deserialize(stream, payload_mass);
    ros::serialization::deserialize(stream, payload_com);
    ros::serialization::deserialize(stream, payload_inertia);
    ros::serialization::deserialize(stream, use_velocities);
    ros::serialization::deserialize(stream, use_accelerations);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(chain);
    size += ros::serialization::serializationLength(payload_mass);
    size += ros::serialization::serializationLength(payload_com);
    size += ros::serialization::serializationLength(payload_inertia);
    size += ros::serialization::serializationLength(use_velocities);
    size += ros::serialization::serializationLength(use_accelerations);
    return size;
  }

  typedef boost::shared_ptr< ::m3_client::M3HumanoidParamRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3_client::M3HumanoidParamRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct M3HumanoidParamRequest
typedef  ::m3_client::M3HumanoidParamRequest_<std::allocator<void> > M3HumanoidParamRequest;

typedef boost::shared_ptr< ::m3_client::M3HumanoidParamRequest> M3HumanoidParamRequestPtr;
typedef boost::shared_ptr< ::m3_client::M3HumanoidParamRequest const> M3HumanoidParamRequestConstPtr;


template <class ContainerAllocator>
struct M3HumanoidParamResponse_ : public ros::Message
{
  typedef M3HumanoidParamResponse_<ContainerAllocator> Type;

  M3HumanoidParamResponse_()
  : response(0)
  {
  }

  M3HumanoidParamResponse_(const ContainerAllocator& _alloc)
  : response(0)
  {
  }

  typedef int32_t _response_type;
  int32_t response;


private:
  static const char* __s_getDataType_() { return "m3_client/M3HumanoidParamResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "f45f68e2feefb1307598e828e260b7d7"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "34cb417585df77f14f1029d4fc16441a"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 response\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, response);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, response);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(response);
    return size;
  }

  typedef boost::shared_ptr< ::m3_client::M3HumanoidParamResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3_client::M3HumanoidParamResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct M3HumanoidParamResponse
typedef  ::m3_client::M3HumanoidParamResponse_<std::allocator<void> > M3HumanoidParamResponse;

typedef boost::shared_ptr< ::m3_client::M3HumanoidParamResponse> M3HumanoidParamResponsePtr;
typedef boost::shared_ptr< ::m3_client::M3HumanoidParamResponse const> M3HumanoidParamResponseConstPtr;

struct M3HumanoidParam
{

typedef M3HumanoidParamRequest Request;
typedef M3HumanoidParamResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct M3HumanoidParam
} // namespace m3_client

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3_client::M3HumanoidParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1bb1c03ca86da994fbea726ff9330f25";
  }

  static const char* value(const  ::m3_client::M3HumanoidParamRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1bb1c03ca86da994ULL;
  static const uint64_t static_value2 = 0xfbea726ff9330f25ULL;
};

template<class ContainerAllocator>
struct DataType< ::m3_client::M3HumanoidParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3HumanoidParamRequest";
  }

  static const char* value(const  ::m3_client::M3HumanoidParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3_client::M3HumanoidParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 chain\n\
float32 payload_mass\n\
float32[3] payload_com\n\
float32[6] payload_inertia\n\
bool use_velocities\n\
bool use_accelerations\n\
\n\
";
  }

  static const char* value(const  ::m3_client::M3HumanoidParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3_client::M3HumanoidParamRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3_client::M3HumanoidParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f45f68e2feefb1307598e828e260b7d7";
  }

  static const char* value(const  ::m3_client::M3HumanoidParamResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf45f68e2feefb130ULL;
  static const uint64_t static_value2 = 0x7598e828e260b7d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::m3_client::M3HumanoidParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3HumanoidParamResponse";
  }

  static const char* value(const  ::m3_client::M3HumanoidParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3_client::M3HumanoidParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 response\n\
\n\
";
  }

  static const char* value(const  ::m3_client::M3HumanoidParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3_client::M3HumanoidParamResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3_client::M3HumanoidParamRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.chain);
    stream.next(m.payload_mass);
    stream.next(m.payload_com);
    stream.next(m.payload_inertia);
    stream.next(m.use_velocities);
    stream.next(m.use_accelerations);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3HumanoidParamRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3_client::M3HumanoidParamResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.response);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3HumanoidParamResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<m3_client::M3HumanoidParam> {
  static const char* value() 
  {
    return "34cb417585df77f14f1029d4fc16441a";
  }

  static const char* value(const m3_client::M3HumanoidParam&) { return value(); } 
};

template<>
struct DataType<m3_client::M3HumanoidParam> {
  static const char* value() 
  {
    return "m3_client/M3HumanoidParam";
  }

  static const char* value(const m3_client::M3HumanoidParam&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3_client::M3HumanoidParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "34cb417585df77f14f1029d4fc16441a";
  }

  static const char* value(const m3_client::M3HumanoidParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3_client::M3HumanoidParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3HumanoidParam";
  }

  static const char* value(const m3_client::M3HumanoidParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3_client::M3HumanoidParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "34cb417585df77f14f1029d4fc16441a";
  }

  static const char* value(const m3_client::M3HumanoidParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3_client::M3HumanoidParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3HumanoidParam";
  }

  static const char* value(const m3_client::M3HumanoidParamResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // M3_CLIENT_SERVICE_M3HUMANOIDPARAM_H

