"""autogenerated by genmsg_py from M3OmnibaseJoy.msg. Do not edit."""
import roslib.message
import struct


class M3OmnibaseJoy(roslib.message.Message):
  _md5sum = "6719502035b93742f7b2585c261584a9"
  _type = "m3_client/M3OmnibaseJoy"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 x
float32 y
float32 yaw
float32 button
float32 z


"""
  __slots__ = ['x','y','yaw','button','z']
  _slot_types = ['float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       x,y,yaw,button,z
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(M3OmnibaseJoy, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.yaw is None:
        self.yaw = 0.
      if self.button is None:
        self.button = 0.
      if self.z is None:
        self.z = 0.
    else:
      self.x = 0.
      self.y = 0.
      self.yaw = 0.
      self.button = 0.
      self.z = 0.

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
      _x = self
      buff.write(_struct_5f.pack(_x.x, _x.y, _x.yaw, _x.button, _x.z))
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
      _x = self
      start = end
      end += 20
      (_x.x, _x.y, _x.yaw, _x.button, _x.z,) = _struct_5f.unpack(str[start:end])
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
      _x = self
      buff.write(_struct_5f.pack(_x.x, _x.y, _x.yaw, _x.button, _x.z))
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
      _x = self
      start = end
      end += 20
      (_x.x, _x.y, _x.yaw, _x.button, _x.z,) = _struct_5f.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_5f = struct.Struct("<5f")
