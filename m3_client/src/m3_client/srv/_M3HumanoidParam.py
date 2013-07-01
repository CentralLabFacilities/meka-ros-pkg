"""autogenerated by genpy from m3_client/M3HumanoidParamRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class M3HumanoidParamRequest(genpy.Message):
  _md5sum = "1bb1c03ca86da994fbea726ff9330f25"
  _type = "m3_client/M3HumanoidParamRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 chain
float32 payload_mass
float32[3] payload_com
float32[6] payload_inertia
bool use_velocities
bool use_accelerations

"""
  __slots__ = ['chain','payload_mass','payload_com','payload_inertia','use_velocities','use_accelerations']
  _slot_types = ['uint8','float32','float32[3]','float32[6]','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       chain,payload_mass,payload_com,payload_inertia,use_velocities,use_accelerations

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(M3HumanoidParamRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.chain is None:
        self.chain = 0
      if self.payload_mass is None:
        self.payload_mass = 0.
      if self.payload_com is None:
        self.payload_com = [0.,0.,0.]
      if self.payload_inertia is None:
        self.payload_inertia = [0.,0.,0.,0.,0.,0.]
      if self.use_velocities is None:
        self.use_velocities = False
      if self.use_accelerations is None:
        self.use_accelerations = False
    else:
      self.chain = 0
      self.payload_mass = 0.
      self.payload_com = [0.,0.,0.]
      self.payload_inertia = [0.,0.,0.,0.,0.,0.]
      self.use_velocities = False
      self.use_accelerations = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_Bf.pack(_x.chain, _x.payload_mass))
      buff.write(_struct_3f.pack(*self.payload_com))
      buff.write(_struct_6f.pack(*self.payload_inertia))
      _x = self
      buff.write(_struct_2B.pack(_x.use_velocities, _x.use_accelerations))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.chain, _x.payload_mass,) = _struct_Bf.unpack(str[start:end])
      start = end
      end += 12
      self.payload_com = _struct_3f.unpack(str[start:end])
      start = end
      end += 24
      self.payload_inertia = _struct_6f.unpack(str[start:end])
      _x = self
      start = end
      end += 2
      (_x.use_velocities, _x.use_accelerations,) = _struct_2B.unpack(str[start:end])
      self.use_velocities = bool(self.use_velocities)
      self.use_accelerations = bool(self.use_accelerations)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_Bf.pack(_x.chain, _x.payload_mass))
      buff.write(self.payload_com.tostring())
      buff.write(self.payload_inertia.tostring())
      _x = self
      buff.write(_struct_2B.pack(_x.use_velocities, _x.use_accelerations))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.chain, _x.payload_mass,) = _struct_Bf.unpack(str[start:end])
      start = end
      end += 12
      self.payload_com = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 24
      self.payload_inertia = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=6)
      _x = self
      start = end
      end += 2
      (_x.use_velocities, _x.use_accelerations,) = _struct_2B.unpack(str[start:end])
      self.use_velocities = bool(self.use_velocities)
      self.use_accelerations = bool(self.use_accelerations)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_Bf = struct.Struct("<Bf")
_struct_2B = struct.Struct("<2B")
_struct_3f = struct.Struct("<3f")
_struct_6f = struct.Struct("<6f")
"""autogenerated by genpy from m3_client/M3HumanoidParamResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class M3HumanoidParamResponse(genpy.Message):
  _md5sum = "f45f68e2feefb1307598e828e260b7d7"
  _type = "m3_client/M3HumanoidParamResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 response

"""
  __slots__ = ['response']
  _slot_types = ['int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       response

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(M3HumanoidParamResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.response is None:
        self.response = 0
    else:
      self.response = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_struct_i.pack(self.response))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (self.response,) = _struct_i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(_struct_i.pack(self.response))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (self.response,) = _struct_i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_i = struct.Struct("<i")
class M3HumanoidParam(object):
  _type          = 'm3_client/M3HumanoidParam'
  _md5sum = '34cb417585df77f14f1029d4fc16441a'
  _request_class  = M3HumanoidParamRequest
  _response_class = M3HumanoidParamResponse