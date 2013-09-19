/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/simple_traj_server/msg/TrajResult.msg */
#ifndef SIMPLE_TRAJ_SERVER_MESSAGE_TRAJRESULT_H
#define SIMPLE_TRAJ_SERVER_MESSAGE_TRAJRESULT_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace simple_traj_server
{
template <class ContainerAllocator>
struct TrajResult_ : public ros::Message
{
  typedef TrajResult_<ContainerAllocator> Type;

  TrajResult_()
  {
  }

  TrajResult_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "simple_traj_server/TrajResult"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    return size;
  }

  typedef boost::shared_ptr< ::simple_traj_server::TrajResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simple_traj_server::TrajResult_<ContainerAllocator>  const> ConstPtr;
}; // struct TrajResult
typedef  ::simple_traj_server::TrajResult_<std::allocator<void> > TrajResult;

typedef boost::shared_ptr< ::simple_traj_server::TrajResult> TrajResultPtr;
typedef boost::shared_ptr< ::simple_traj_server::TrajResult const> TrajResultConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::simple_traj_server::TrajResult_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::simple_traj_server::TrajResult_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace simple_traj_server

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::simple_traj_server::TrajResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::simple_traj_server::TrajResult_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::simple_traj_server::TrajResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "simple_traj_server/TrajResult";
  }

  static const char* value(const  ::simple_traj_server::TrajResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::simple_traj_server::TrajResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
";
  }

  static const char* value(const  ::simple_traj_server::TrajResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::simple_traj_server::TrajResult_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::simple_traj_server::TrajResult_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct TrajResult_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::simple_traj_server::TrajResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::simple_traj_server::TrajResult_<ContainerAllocator> & v) 
  {
  }
};


} // namespace message_operations
} // namespace ros

#endif // SIMPLE_TRAJ_SERVER_MESSAGE_TRAJRESULT_H
