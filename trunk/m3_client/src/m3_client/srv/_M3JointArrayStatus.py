"""autogenerated by genmsg_py from M3JointArrayStatusRequest.msg. Do not edit."""
import roslib.message
import struct


class M3JointArrayStatusRequest(roslib.message.Message):
  _md5sum = "650f0ccd41c8f8d53ada80be6ddde434"
  _type = "m3_client/M3JointArrayStatusRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 request

"""
  __slots__ = ['request']
  _slot_types = ['int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       request
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(M3JointArrayStatusRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.request is None:
        self.request = 0
    else:
      self.request = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      buff.write(_struct_i.pack(self.request))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 4
      (self.request,) = _struct_i.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      buff.write(_struct_i.pack(self.request))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      start = end
      end += 4
      (self.request,) = _struct_i.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_i = struct.Struct("<i")
"""autogenerated by genmsg_py from M3JointArrayStatusResponse.msg. Do not edit."""
import roslib.message
import struct

import m3_client.msg

class M3JointArrayStatusResponse(roslib.message.Message):
  _md5sum = "b3c51b6c0fba9e6bcb03e3e871454442"
  _type = "m3_client/M3JointArrayStatusResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """M3BaseStatus base
float32[] motor_temp
float32[] amp_temp
float32[] current
float32[] torque
float32[] torquedot
float32[] theta
float32[] thetadot
float32[] thetadotdot
int32 completed_spline_idx
int32[] pwm_cmd

================================================================================
MSG: m3_client/M3BaseStatus
string name
uint8 state
int64 timestamp
string rate
string version


"""
  __slots__ = ['base','motor_temp','amp_temp','current','torque','torquedot','theta','thetadot','thetadotdot','completed_spline_idx','pwm_cmd']
  _slot_types = ['m3_client/M3BaseStatus','float32[]','float32[]','float32[]','float32[]','float32[]','float32[]','float32[]','float32[]','int32','int32[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       base,motor_temp,amp_temp,current,torque,torquedot,theta,thetadot,thetadotdot,completed_spline_idx,pwm_cmd
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(M3JointArrayStatusResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.base is None:
        self.base = m3_client.msg.M3BaseStatus()
      if self.motor_temp is None:
        self.motor_temp = []
      if self.amp_temp is None:
        self.amp_temp = []
      if self.current is None:
        self.current = []
      if self.torque is None:
        self.torque = []
      if self.torquedot is None:
        self.torquedot = []
      if self.theta is None:
        self.theta = []
      if self.thetadot is None:
        self.thetadot = []
      if self.thetadotdot is None:
        self.thetadotdot = []
      if self.completed_spline_idx is None:
        self.completed_spline_idx = 0
      if self.pwm_cmd is None:
        self.pwm_cmd = []
    else:
      self.base = m3_client.msg.M3BaseStatus()
      self.motor_temp = []
      self.amp_temp = []
      self.current = []
      self.torque = []
      self.torquedot = []
      self.theta = []
      self.thetadot = []
      self.thetadotdot = []
      self.completed_spline_idx = 0
      self.pwm_cmd = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self.base.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_Bq.pack(_x.base.state, _x.base.timestamp))
      _x = self.base.rate
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.base.version
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.motor_temp)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.motor_temp))
      length = len(self.amp_temp)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.amp_temp))
      length = len(self.current)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.current))
      length = len(self.torque)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.torque))
      length = len(self.torquedot)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.torquedot))
      length = len(self.theta)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.theta))
      length = len(self.thetadot)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.thetadot))
      length = len(self.thetadotdot)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.thetadotdot))
      buff.write(_struct_i.pack(self.completed_spline_idx))
      length = len(self.pwm_cmd)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.pwm_cmd))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.base is None:
        self.base = m3_client.msg.M3BaseStatus()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.base.name = str[start:end]
      _x = self
      start = end
      end += 9
      (_x.base.state, _x.base.timestamp,) = _struct_Bq.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.base.rate = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.base.version = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.motor_temp = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.amp_temp = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.current = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.torque = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.torquedot = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.theta = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.thetadot = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.thetadotdot = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (self.completed_spline_idx,) = _struct_i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.pwm_cmd = struct.unpack(pattern, str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self.base.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_Bq.pack(_x.base.state, _x.base.timestamp))
      _x = self.base.rate
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.base.version
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.motor_temp)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.motor_temp.tostring())
      length = len(self.amp_temp)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.amp_temp.tostring())
      length = len(self.current)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.current.tostring())
      length = len(self.torque)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.torque.tostring())
      length = len(self.torquedot)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.torquedot.tostring())
      length = len(self.theta)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.theta.tostring())
      length = len(self.thetadot)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.thetadot.tostring())
      length = len(self.thetadotdot)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.thetadotdot.tostring())
      buff.write(_struct_i.pack(self.completed_spline_idx))
      length = len(self.pwm_cmd)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.pwm_cmd.tostring())
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.base is None:
        self.base = m3_client.msg.M3BaseStatus()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.base.name = str[start:end]
      _x = self
      start = end
      end += 9
      (_x.base.state, _x.base.timestamp,) = _struct_Bq.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.base.rate = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.base.version = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.motor_temp = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.amp_temp = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.current = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.torque = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.torquedot = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.theta = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.thetadot = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.thetadotdot = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (self.completed_spline_idx,) = _struct_i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.pwm_cmd = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_i = struct.Struct("<i")
_struct_Bq = struct.Struct("<Bq")
class M3JointArrayStatus(roslib.message.ServiceDefinition):
  _type          = 'm3_client/M3JointArrayStatus'
  _md5sum = 'fd0ff7bf51cf3c5324e0ee2aef6015d7'
  _request_class  = M3JointArrayStatusRequest
  _response_class = M3JointArrayStatusResponse
