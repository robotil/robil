"""autogenerated by genpy from C11_Agent/override_object_propertiesRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import C11_Agent.msg

class override_object_propertiesRequest(genpy.Message):
  _md5sum = "80a5a708c1560a51133202325435fc54"
  _type = "C11_Agent/override_object_propertiesRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """C11_Agent/C11C23_OBP   obp


================================================================================
MSG: C11_Agent/C11C23_OBP
int16 ACT
int16 ACT_MODIFIED = 0
int16 ACT_NEW      = 1
int16 FRZ
int16 FRZ_KEEP  = 0
int16 ACT_RETRY = 1
int16 ID
string NAME
C11_Agent/pathLocation[] EXTR_VIS
C11_Agent/pathLocation[] EXTR_TOP
C11_Agent/D3SPACE ORI
================================================================================
MSG: C11_Agent/pathLocation
float64 lat
float64 lon
================================================================================
MSG: C11_Agent/D3SPACE
float64 ROLL
float64 PITCH
float64 YAW
"""
  __slots__ = ['obp']
  _slot_types = ['C11_Agent/C11C23_OBP']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       obp

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(override_object_propertiesRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.obp is None:
        self.obp = C11_Agent.msg.C11C23_OBP()
    else:
      self.obp = C11_Agent.msg.C11C23_OBP()

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
      buff.write(_struct_3h.pack(_x.obp.ACT, _x.obp.FRZ, _x.obp.ID))
      _x = self.obp.NAME
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.obp.EXTR_VIS)
      buff.write(_struct_I.pack(length))
      for val1 in self.obp.EXTR_VIS:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.obp.EXTR_TOP)
      buff.write(_struct_I.pack(length))
      for val1 in self.obp.EXTR_TOP:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      _x = self
      buff.write(_struct_3d.pack(_x.obp.ORI.ROLL, _x.obp.ORI.PITCH, _x.obp.ORI.YAW))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.obp is None:
        self.obp = C11_Agent.msg.C11C23_OBP()
      end = 0
      _x = self
      start = end
      end += 6
      (_x.obp.ACT, _x.obp.FRZ, _x.obp.ID,) = _struct_3h.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.obp.NAME = str[start:end].decode('utf-8')
      else:
        self.obp.NAME = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obp.EXTR_VIS = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.obp.EXTR_VIS.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obp.EXTR_TOP = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.obp.EXTR_TOP.append(val1)
      _x = self
      start = end
      end += 24
      (_x.obp.ORI.ROLL, _x.obp.ORI.PITCH, _x.obp.ORI.YAW,) = _struct_3d.unpack(str[start:end])
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
      buff.write(_struct_3h.pack(_x.obp.ACT, _x.obp.FRZ, _x.obp.ID))
      _x = self.obp.NAME
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.obp.EXTR_VIS)
      buff.write(_struct_I.pack(length))
      for val1 in self.obp.EXTR_VIS:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.obp.EXTR_TOP)
      buff.write(_struct_I.pack(length))
      for val1 in self.obp.EXTR_TOP:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      _x = self
      buff.write(_struct_3d.pack(_x.obp.ORI.ROLL, _x.obp.ORI.PITCH, _x.obp.ORI.YAW))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.obp is None:
        self.obp = C11_Agent.msg.C11C23_OBP()
      end = 0
      _x = self
      start = end
      end += 6
      (_x.obp.ACT, _x.obp.FRZ, _x.obp.ID,) = _struct_3h.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.obp.NAME = str[start:end].decode('utf-8')
      else:
        self.obp.NAME = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obp.EXTR_VIS = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.obp.EXTR_VIS.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obp.EXTR_TOP = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.obp.EXTR_TOP.append(val1)
      _x = self
      start = end
      end += 24
      (_x.obp.ORI.ROLL, _x.obp.ORI.PITCH, _x.obp.ORI.YAW,) = _struct_3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2d = struct.Struct("<2d")
_struct_3h = struct.Struct("<3h")
_struct_3d = struct.Struct("<3d")
"""autogenerated by genpy from C11_Agent/override_object_propertiesResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class override_object_propertiesResponse(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "C11_Agent/override_object_propertiesResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """



"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(override_object_propertiesResponse, self).__init__(*args, **kwds)

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
      pass
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
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
      pass
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
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
class override_object_properties(object):
  _type          = 'C11_Agent/override_object_properties'
  _md5sum = '80a5a708c1560a51133202325435fc54'
  _request_class  = override_object_propertiesRequest
  _response_class = override_object_propertiesResponse
