"""autogenerated by genpy from m3_client/M3LoadX6StatusRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class M3LoadX6StatusRequest(genpy.Message):
  _md5sum = "650f0ccd41c8f8d53ada80be6ddde434"
  _type = "m3_client/M3LoadX6StatusRequest"
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

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(M3LoadX6StatusRequest, self).__init__(*args, **kwds)
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
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_struct_i.pack(self.request))
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
      (self.request,) = _struct_i.unpack(str[start:end])
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
      buff.write(_struct_i.pack(self.request))
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
      (self.request,) = _struct_i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_i = struct.Struct("<i")
"""autogenerated by genpy from m3_client/M3LoadX6StatusResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import m3_client.msg

class M3LoadX6StatusResponse(genpy.Message):
  _md5sum = "2990baf3156c3c6b74caf36efa6995dd"
  _type = "m3_client/M3LoadX6StatusResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """M3BaseStatus base
float32[6] wrench
float32 adc_ext_0
float32 adc_ext_1
float32 adc_ext_2
float32 dig_ext_0


================================================================================
MSG: m3_client/M3BaseStatus
string name
uint8 state
int64 timestamp
string rate
string version


"""
  __slots__ = ['base','wrench','adc_ext_0','adc_ext_1','adc_ext_2','dig_ext_0']
  _slot_types = ['m3_client/M3BaseStatus','float32[6]','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       base,wrench,adc_ext_0,adc_ext_1,adc_ext_2,dig_ext_0

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(M3LoadX6StatusResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.base is None:
        self.base = m3_client.msg.M3BaseStatus()
      if self.wrench is None:
        self.wrench = [0.,0.,0.,0.,0.,0.]
      if self.adc_ext_0 is None:
        self.adc_ext_0 = 0.
      if self.adc_ext_1 is None:
        self.adc_ext_1 = 0.
      if self.adc_ext_2 is None:
        self.adc_ext_2 = 0.
      if self.dig_ext_0 is None:
        self.dig_ext_0 = 0.
    else:
      self.base = m3_client.msg.M3BaseStatus()
      self.wrench = [0.,0.,0.,0.,0.,0.]
      self.adc_ext_0 = 0.
      self.adc_ext_1 = 0.
      self.adc_ext_2 = 0.
      self.dig_ext_0 = 0.

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
      _x = self.base.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_Bq.pack(_x.base.state, _x.base.timestamp))
      _x = self.base.rate
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.base.version
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_6f.pack(*self.wrench))
      _x = self
      buff.write(_struct_4f.pack(_x.adc_ext_0, _x.adc_ext_1, _x.adc_ext_2, _x.dig_ext_0))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
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
      if python3:
        self.base.name = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.base.rate = str[start:end].decode('utf-8')
      else:
        self.base.rate = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.base.version = str[start:end].decode('utf-8')
      else:
        self.base.version = str[start:end]
      start = end
      end += 24
      self.wrench = _struct_6f.unpack(str[start:end])
      _x = self
      start = end
      end += 16
      (_x.adc_ext_0, _x.adc_ext_1, _x.adc_ext_2, _x.dig_ext_0,) = _struct_4f.unpack(str[start:end])
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
      _x = self.base.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_Bq.pack(_x.base.state, _x.base.timestamp))
      _x = self.base.rate
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.base.version
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(self.wrench.tostring())
      _x = self
      buff.write(_struct_4f.pack(_x.adc_ext_0, _x.adc_ext_1, _x.adc_ext_2, _x.dig_ext_0))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
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
      if python3:
        self.base.name = str[start:end].decode('utf-8')
      else:
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
      if python3:
        self.base.rate = str[start:end].decode('utf-8')
      else:
        self.base.rate = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.base.version = str[start:end].decode('utf-8')
      else:
        self.base.version = str[start:end]
      start = end
      end += 24
      self.wrench = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=6)
      _x = self
      start = end
      end += 16
      (_x.adc_ext_0, _x.adc_ext_1, _x.adc_ext_2, _x.dig_ext_0,) = _struct_4f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4f = struct.Struct("<4f")
_struct_Bq = struct.Struct("<Bq")
_struct_6f = struct.Struct("<6f")
class M3LoadX6Status(object):
  _type          = 'm3_client/M3LoadX6Status'
  _md5sum = '967558453e196f040b12e56469a00597'
  _request_class  = M3LoadX6StatusRequest
  _response_class = M3LoadX6StatusResponse