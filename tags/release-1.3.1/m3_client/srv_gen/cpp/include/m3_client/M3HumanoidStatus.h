/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/meka-ros-pkg/m3_client/srv/M3HumanoidStatus.srv */
#ifndef M3_CLIENT_SERVICE_M3HUMANOIDSTATUS_H
#define M3_CLIENT_SERVICE_M3HUMANOIDSTATUS_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"



#include "m3_client/M3BaseStatus.h"

namespace m3_client
{
template <class ContainerAllocator>
struct M3HumanoidStatusRequest_ : public ros::Message
{
  typedef M3HumanoidStatusRequest_<ContainerAllocator> Type;

  M3HumanoidStatusRequest_()
  : chain(0)
  {
  }

  M3HumanoidStatusRequest_(const ContainerAllocator& _alloc)
  : chain(0)
  {
  }

  typedef uint8_t _chain_type;
  uint8_t chain;


private:
  static const char* __s_getDataType_() { return "m3_client/M3HumanoidStatusRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "0ffb919411971afc3767d003eaba94ea"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "7ccd98e6ad058fad2be7361cfc7d0728"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "uint8 chain\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, chain);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, chain);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(chain);
    return size;
  }

  typedef boost::shared_ptr< ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct M3HumanoidStatusRequest
typedef  ::m3_client::M3HumanoidStatusRequest_<std::allocator<void> > M3HumanoidStatusRequest;

typedef boost::shared_ptr< ::m3_client::M3HumanoidStatusRequest> M3HumanoidStatusRequestPtr;
typedef boost::shared_ptr< ::m3_client::M3HumanoidStatusRequest const> M3HumanoidStatusRequestConstPtr;


template <class ContainerAllocator>
struct M3HumanoidStatusResponse_ : public ros::Message
{
  typedef M3HumanoidStatusResponse_<ContainerAllocator> Type;

  M3HumanoidStatusResponse_()
  : base()
  , joint_names()
  , torque()
  , torquedot()
  , theta()
  , thetadot()
  , thetadotdot()
  , completed_spline_idx(0)
  , end_pos()
  , end_rot()
  , J()
  , G()
  , end_twist()
  , pwm_cmd()
  , motor_enabled(false)
  {
    end_pos.assign(0.0);
    end_rot.assign(0.0);
    end_twist.assign(0.0);
  }

  M3HumanoidStatusResponse_(const ContainerAllocator& _alloc)
  : base(_alloc)
  , joint_names(_alloc)
  , torque(_alloc)
  , torquedot(_alloc)
  , theta(_alloc)
  , thetadot(_alloc)
  , thetadotdot(_alloc)
  , completed_spline_idx(0)
  , end_pos()
  , end_rot()
  , J(_alloc)
  , G(_alloc)
  , end_twist()
  , pwm_cmd(_alloc)
  , motor_enabled(false)
  {
    end_pos.assign(0.0);
    end_rot.assign(0.0);
    end_twist.assign(0.0);
  }

  typedef  ::m3_client::M3BaseStatus_<ContainerAllocator>  _base_type;
   ::m3_client::M3BaseStatus_<ContainerAllocator>  base;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _joint_names_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  joint_names;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _torque_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  torque;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _torquedot_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  torquedot;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _theta_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  theta;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _thetadot_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  thetadot;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _thetadotdot_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  thetadotdot;

  typedef int32_t _completed_spline_idx_type;
  int32_t completed_spline_idx;

  typedef boost::array<float, 3>  _end_pos_type;
  boost::array<float, 3>  end_pos;

  typedef boost::array<float, 9>  _end_rot_type;
  boost::array<float, 9>  end_rot;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _J_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  J;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _G_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  G;

  typedef boost::array<float, 6>  _end_twist_type;
  boost::array<float, 6>  end_twist;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _pwm_cmd_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  pwm_cmd;

  typedef uint8_t _motor_enabled_type;
  uint8_t motor_enabled;


  ROSCPP_DEPRECATED uint32_t get_joint_names_size() const { return (uint32_t)joint_names.size(); }
  ROSCPP_DEPRECATED void set_joint_names_size(uint32_t size) { joint_names.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_joint_names_vec(std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other > & vec) const { vec = this->joint_names; }
  ROSCPP_DEPRECATED void set_joint_names_vec(const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other > & vec) { this->joint_names = vec; }
  ROSCPP_DEPRECATED uint32_t get_torque_size() const { return (uint32_t)torque.size(); }
  ROSCPP_DEPRECATED void set_torque_size(uint32_t size) { torque.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_torque_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->torque; }
  ROSCPP_DEPRECATED void set_torque_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->torque = vec; }
  ROSCPP_DEPRECATED uint32_t get_torquedot_size() const { return (uint32_t)torquedot.size(); }
  ROSCPP_DEPRECATED void set_torquedot_size(uint32_t size) { torquedot.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_torquedot_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->torquedot; }
  ROSCPP_DEPRECATED void set_torquedot_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->torquedot = vec; }
  ROSCPP_DEPRECATED uint32_t get_theta_size() const { return (uint32_t)theta.size(); }
  ROSCPP_DEPRECATED void set_theta_size(uint32_t size) { theta.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_theta_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->theta; }
  ROSCPP_DEPRECATED void set_theta_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->theta = vec; }
  ROSCPP_DEPRECATED uint32_t get_thetadot_size() const { return (uint32_t)thetadot.size(); }
  ROSCPP_DEPRECATED void set_thetadot_size(uint32_t size) { thetadot.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_thetadot_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->thetadot; }
  ROSCPP_DEPRECATED void set_thetadot_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->thetadot = vec; }
  ROSCPP_DEPRECATED uint32_t get_thetadotdot_size() const { return (uint32_t)thetadotdot.size(); }
  ROSCPP_DEPRECATED void set_thetadotdot_size(uint32_t size) { thetadotdot.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_thetadotdot_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->thetadotdot; }
  ROSCPP_DEPRECATED void set_thetadotdot_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->thetadotdot = vec; }
  ROSCPP_DEPRECATED uint32_t get_end_pos_size() const { return (uint32_t)end_pos.size(); }
  ROSCPP_DEPRECATED uint32_t get_end_rot_size() const { return (uint32_t)end_rot.size(); }
  ROSCPP_DEPRECATED uint32_t get_J_size() const { return (uint32_t)J.size(); }
  ROSCPP_DEPRECATED void set_J_size(uint32_t size) { J.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_J_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->J; }
  ROSCPP_DEPRECATED void set_J_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->J = vec; }
  ROSCPP_DEPRECATED uint32_t get_G_size() const { return (uint32_t)G.size(); }
  ROSCPP_DEPRECATED void set_G_size(uint32_t size) { G.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_G_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->G; }
  ROSCPP_DEPRECATED void set_G_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->G = vec; }
  ROSCPP_DEPRECATED uint32_t get_end_twist_size() const { return (uint32_t)end_twist.size(); }
  ROSCPP_DEPRECATED uint32_t get_pwm_cmd_size() const { return (uint32_t)pwm_cmd.size(); }
  ROSCPP_DEPRECATED void set_pwm_cmd_size(uint32_t size) { pwm_cmd.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_pwm_cmd_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->pwm_cmd; }
  ROSCPP_DEPRECATED void set_pwm_cmd_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->pwm_cmd = vec; }
private:
  static const char* __s_getDataType_() { return "m3_client/M3HumanoidStatusResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "a6a6f9dfa781956d0e52a3c86bfd52a5"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "7ccd98e6ad058fad2be7361cfc7d0728"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "M3BaseStatus base\n\
string[] joint_names\n\
float32[] torque\n\
float32[] torquedot\n\
float32[] theta\n\
float32[] thetadot\n\
float32[] thetadotdot\n\
int32 completed_spline_idx\n\
float32[3] end_pos\n\
float32[9] end_rot\n\
float32[] J\n\
float32[] G\n\
float32[6] end_twist\n\
float32[] pwm_cmd\n\
bool motor_enabled\n\
\n\
================================================================================\n\
MSG: m3_client/M3BaseStatus\n\
string name\n\
uint8 state\n\
int64 timestamp\n\
string rate\n\
string version\n\
\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, base);
    ros::serialization::serialize(stream, joint_names);
    ros::serialization::serialize(stream, torque);
    ros::serialization::serialize(stream, torquedot);
    ros::serialization::serialize(stream, theta);
    ros::serialization::serialize(stream, thetadot);
    ros::serialization::serialize(stream, thetadotdot);
    ros::serialization::serialize(stream, completed_spline_idx);
    ros::serialization::serialize(stream, end_pos);
    ros::serialization::serialize(stream, end_rot);
    ros::serialization::serialize(stream, J);
    ros::serialization::serialize(stream, G);
    ros::serialization::serialize(stream, end_twist);
    ros::serialization::serialize(stream, pwm_cmd);
    ros::serialization::serialize(stream, motor_enabled);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, base);
    ros::serialization::deserialize(stream, joint_names);
    ros::serialization::deserialize(stream, torque);
    ros::serialization::deserialize(stream, torquedot);
    ros::serialization::deserialize(stream, theta);
    ros::serialization::deserialize(stream, thetadot);
    ros::serialization::deserialize(stream, thetadotdot);
    ros::serialization::deserialize(stream, completed_spline_idx);
    ros::serialization::deserialize(stream, end_pos);
    ros::serialization::deserialize(stream, end_rot);
    ros::serialization::deserialize(stream, J);
    ros::serialization::deserialize(stream, G);
    ros::serialization::deserialize(stream, end_twist);
    ros::serialization::deserialize(stream, pwm_cmd);
    ros::serialization::deserialize(stream, motor_enabled);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(base);
    size += ros::serialization::serializationLength(joint_names);
    size += ros::serialization::serializationLength(torque);
    size += ros::serialization::serializationLength(torquedot);
    size += ros::serialization::serializationLength(theta);
    size += ros::serialization::serializationLength(thetadot);
    size += ros::serialization::serializationLength(thetadotdot);
    size += ros::serialization::serializationLength(completed_spline_idx);
    size += ros::serialization::serializationLength(end_pos);
    size += ros::serialization::serializationLength(end_rot);
    size += ros::serialization::serializationLength(J);
    size += ros::serialization::serializationLength(G);
    size += ros::serialization::serializationLength(end_twist);
    size += ros::serialization::serializationLength(pwm_cmd);
    size += ros::serialization::serializationLength(motor_enabled);
    return size;
  }

  typedef boost::shared_ptr< ::m3_client::M3HumanoidStatusResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3_client::M3HumanoidStatusResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct M3HumanoidStatusResponse
typedef  ::m3_client::M3HumanoidStatusResponse_<std::allocator<void> > M3HumanoidStatusResponse;

typedef boost::shared_ptr< ::m3_client::M3HumanoidStatusResponse> M3HumanoidStatusResponsePtr;
typedef boost::shared_ptr< ::m3_client::M3HumanoidStatusResponse const> M3HumanoidStatusResponseConstPtr;

struct M3HumanoidStatus
{

typedef M3HumanoidStatusRequest Request;
typedef M3HumanoidStatusResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct M3HumanoidStatus
} // namespace m3_client

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0ffb919411971afc3767d003eaba94ea";
  }

  static const char* value(const  ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0ffb919411971afcULL;
  static const uint64_t static_value2 = 0x3767d003eaba94eaULL;
};

template<class ContainerAllocator>
struct DataType< ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3HumanoidStatusRequest";
  }

  static const char* value(const  ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 chain\n\
\n\
";
  }

  static const char* value(const  ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3_client::M3HumanoidStatusResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a6a6f9dfa781956d0e52a3c86bfd52a5";
  }

  static const char* value(const  ::m3_client::M3HumanoidStatusResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa6a6f9dfa781956dULL;
  static const uint64_t static_value2 = 0x0e52a3c86bfd52a5ULL;
};

template<class ContainerAllocator>
struct DataType< ::m3_client::M3HumanoidStatusResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3HumanoidStatusResponse";
  }

  static const char* value(const  ::m3_client::M3HumanoidStatusResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3_client::M3HumanoidStatusResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "M3BaseStatus base\n\
string[] joint_names\n\
float32[] torque\n\
float32[] torquedot\n\
float32[] theta\n\
float32[] thetadot\n\
float32[] thetadotdot\n\
int32 completed_spline_idx\n\
float32[3] end_pos\n\
float32[9] end_rot\n\
float32[] J\n\
float32[] G\n\
float32[6] end_twist\n\
float32[] pwm_cmd\n\
bool motor_enabled\n\
\n\
================================================================================\n\
MSG: m3_client/M3BaseStatus\n\
string name\n\
uint8 state\n\
int64 timestamp\n\
string rate\n\
string version\n\
\n\
\n\
";
  }

  static const char* value(const  ::m3_client::M3HumanoidStatusResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3_client::M3HumanoidStatusRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.chain);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3HumanoidStatusRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3_client::M3HumanoidStatusResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.base);
    stream.next(m.joint_names);
    stream.next(m.torque);
    stream.next(m.torquedot);
    stream.next(m.theta);
    stream.next(m.thetadot);
    stream.next(m.thetadotdot);
    stream.next(m.completed_spline_idx);
    stream.next(m.end_pos);
    stream.next(m.end_rot);
    stream.next(m.J);
    stream.next(m.G);
    stream.next(m.end_twist);
    stream.next(m.pwm_cmd);
    stream.next(m.motor_enabled);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3HumanoidStatusResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<m3_client::M3HumanoidStatus> {
  static const char* value() 
  {
    return "7ccd98e6ad058fad2be7361cfc7d0728";
  }

  static const char* value(const m3_client::M3HumanoidStatus&) { return value(); } 
};

template<>
struct DataType<m3_client::M3HumanoidStatus> {
  static const char* value() 
  {
    return "m3_client/M3HumanoidStatus";
  }

  static const char* value(const m3_client::M3HumanoidStatus&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3_client::M3HumanoidStatusRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7ccd98e6ad058fad2be7361cfc7d0728";
  }

  static const char* value(const m3_client::M3HumanoidStatusRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3_client::M3HumanoidStatusRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3HumanoidStatus";
  }

  static const char* value(const m3_client::M3HumanoidStatusRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3_client::M3HumanoidStatusResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7ccd98e6ad058fad2be7361cfc7d0728";
  }

  static const char* value(const m3_client::M3HumanoidStatusResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3_client::M3HumanoidStatusResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_client/M3HumanoidStatus";
  }

  static const char* value(const m3_client::M3HumanoidStatusResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // M3_CLIENT_SERVICE_M3HUMANOIDSTATUS_H

