"""autogenerated by genpy from C11_Agent/override_obstacle_propertiesRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import C11_Agent.msg

class override_obstacle_propertiesRequest(genpy.Message):
  _md5sum = "86960dd616b9dfcf6d09f709fea7366c"
  _type = "C11_Agent/override_obstacle_propertiesRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """C11_Agent/C11C24_OSP   osp

================================================================================
MSG: C11_Agent/C11C24_OSP
int32 ACT_TYPE
int32 ACT_TYPE_MODIFIED = 0
int32 ACT_TYPE_DELETED  = 1
int32 ACT_TYPE_NEW      = 2
int32 FRZ
int32 FRZ_KEEP  = 0
int32 FRZ_RETRY = 1
int32 TYPE
int32 TYPE_STATIC=0
int32 TYPE_DYNAMIC=1
int16 ID
C11_Agent/pathLocation[] EXTR_VIS
C11_Agent/pathLocation[] EXTR_TOP
C11_Agent/pathLocation[] IGNR
================================================================================
MSG: C11_Agent/pathLocation
float64 lat
float64 lon
"""
  __slots__ = ['osp']
  _slot_types = ['C11_Agent/C11C24_OSP']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       osp

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(override_obstacle_propertiesRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.osp is None:
        self.osp = C11_Agent.msg.C11C24_OSP()
    else:
      self.osp = C11_Agent.msg.C11C24_OSP()

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
      buff.write(_struct_3ih.pack(_x.osp.ACT_TYPE, _x.osp.FRZ, _x.osp.TYPE, _x.osp.ID))
      length = len(self.osp.EXTR_VIS)
      buff.write(_struct_I.pack(length))
      for val1 in self.osp.EXTR_VIS:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.osp.EXTR_TOP)
      buff.write(_struct_I.pack(length))
      for val1 in self.osp.EXTR_TOP:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.osp.IGNR)
      buff.write(_struct_I.pack(length))
      for val1 in self.osp.IGNR:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.osp is None:
        self.osp = C11_Agent.msg.C11C24_OSP()
      end = 0
      _x = self
      start = end
      end += 14
      (_x.osp.ACT_TYPE, _x.osp.FRZ, _x.osp.TYPE, _x.osp.ID,) = _struct_3ih.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osp.EXTR_VIS = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osp.EXTR_VIS.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osp.EXTR_TOP = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osp.EXTR_TOP.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osp.IGNR = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osp.IGNR.append(val1)
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
      buff.write(_struct_3ih.pack(_x.osp.ACT_TYPE, _x.osp.FRZ, _x.osp.TYPE, _x.osp.ID))
      length = len(self.osp.EXTR_VIS)
      buff.write(_struct_I.pack(length))
      for val1 in self.osp.EXTR_VIS:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.osp.EXTR_TOP)
      buff.write(_struct_I.pack(length))
      for val1 in self.osp.EXTR_TOP:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.osp.IGNR)
      buff.write(_struct_I.pack(length))
      for val1 in self.osp.IGNR:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.osp is None:
        self.osp = C11_Agent.msg.C11C24_OSP()
      end = 0
      _x = self
      start = end
      end += 14
      (_x.osp.ACT_TYPE, _x.osp.FRZ, _x.osp.TYPE, _x.osp.ID,) = _struct_3ih.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osp.EXTR_VIS = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osp.EXTR_VIS.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osp.EXTR_TOP = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osp.EXTR_TOP.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osp.IGNR = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osp.IGNR.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2d = struct.Struct("<2d")
_struct_3ih = struct.Struct("<3ih")
"""autogenerated by genpy from C11_Agent/override_obstacle_propertiesResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class override_obstacle_propertiesResponse(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "C11_Agent/override_obstacle_propertiesResponse"
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
      super(override_obstacle_propertiesResponse, self).__init__(*args, **kwds)

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
class override_obstacle_properties(object):
  _type          = 'C11_Agent/override_obstacle_properties'
  _md5sum = '86960dd616b9dfcf6d09f709fea7366c'
  _request_class  = override_obstacle_propertiesRequest
  _response_class = override_obstacle_propertiesResponse
