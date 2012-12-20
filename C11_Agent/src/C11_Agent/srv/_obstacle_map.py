"""autogenerated by genpy from C11_Agent/obstacle_mapRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import C11_Agent.msg

class obstacle_mapRequest(genpy.Message):
  _md5sum = "6cd8edbdff95b85b642268471c7b5943"
  _type = "C11_Agent/obstacle_mapRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
C11_Agent/C11C24_OSM  osm


================================================================================
MSG: C11_Agent/C11C24_OSM
int16 TYP
int16 TYP_STATIC  = 0
int16 TYP_DYNAMIC = 1
float64 LAT
int16 SCN
int16 SCN_SCAN    = 0
int16 SCN_CURRENT = 1
int16 MOV
int16 MOV_NONE     = 0
int16 MOV_HEAD     = 1
int16 MOV_POSTURE  = 2
int16 MOV_POSITION = 3
"""
  __slots__ = ['osm']
  _slot_types = ['C11_Agent/C11C24_OSM']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       osm

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(obstacle_mapRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.osm is None:
        self.osm = C11_Agent.msg.C11C24_OSM()
    else:
      self.osm = C11_Agent.msg.C11C24_OSM()

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
      buff.write(_struct_hd2h.pack(_x.osm.TYP, _x.osm.LAT, _x.osm.SCN, _x.osm.MOV))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.osm is None:
        self.osm = C11_Agent.msg.C11C24_OSM()
      end = 0
      _x = self
      start = end
      end += 14
      (_x.osm.TYP, _x.osm.LAT, _x.osm.SCN, _x.osm.MOV,) = _struct_hd2h.unpack(str[start:end])
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
      buff.write(_struct_hd2h.pack(_x.osm.TYP, _x.osm.LAT, _x.osm.SCN, _x.osm.MOV))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.osm is None:
        self.osm = C11_Agent.msg.C11C24_OSM()
      end = 0
      _x = self
      start = end
      end += 14
      (_x.osm.TYP, _x.osm.LAT, _x.osm.SCN, _x.osm.MOV,) = _struct_hd2h.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_hd2h = struct.Struct("<hd2h")
"""autogenerated by genpy from C11_Agent/obstacle_mapResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import C11_Agent.msg

class obstacle_mapResponse(genpy.Message):
  _md5sum = "21b5dfa4e4a98c8a4bae2b03dc1bf029"
  _type = "C11_Agent/obstacle_mapResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """C11_Agent/C24C11_OSM  osm


================================================================================
MSG: C11_Agent/C24C11_OSM
int32 TYPE
int32 TYPE_STATIC  = 0
int32 TYPE_DYNAMIC = 1
int16 ID
C11_Agent/coord[] COM
C11_Agent/pathLocation[] EXTR_VIS
C11_Agent/pathLocation[] EXTR_TOP
C11_Agent/coord EXTR_3D
================================================================================
MSG: C11_Agent/coord
float64 X
float64 Y
float64 Z
================================================================================
MSG: C11_Agent/pathLocation
float64 lat
float64 lon
"""
  __slots__ = ['osm']
  _slot_types = ['C11_Agent/C24C11_OSM']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       osm

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(obstacle_mapResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.osm is None:
        self.osm = C11_Agent.msg.C24C11_OSM()
    else:
      self.osm = C11_Agent.msg.C24C11_OSM()

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
      buff.write(_struct_ih.pack(_x.osm.TYPE, _x.osm.ID))
      length = len(self.osm.COM)
      buff.write(_struct_I.pack(length))
      for val1 in self.osm.COM:
        _x = val1
        buff.write(_struct_3d.pack(_x.X, _x.Y, _x.Z))
      length = len(self.osm.EXTR_VIS)
      buff.write(_struct_I.pack(length))
      for val1 in self.osm.EXTR_VIS:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.osm.EXTR_TOP)
      buff.write(_struct_I.pack(length))
      for val1 in self.osm.EXTR_TOP:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      _x = self
      buff.write(_struct_3d.pack(_x.osm.EXTR_3D.X, _x.osm.EXTR_3D.Y, _x.osm.EXTR_3D.Z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.osm is None:
        self.osm = C11_Agent.msg.C24C11_OSM()
      end = 0
      _x = self
      start = end
      end += 6
      (_x.osm.TYPE, _x.osm.ID,) = _struct_ih.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osm.COM = []
      for i in range(0, length):
        val1 = C11_Agent.msg.coord()
        _x = val1
        start = end
        end += 24
        (_x.X, _x.Y, _x.Z,) = _struct_3d.unpack(str[start:end])
        self.osm.COM.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osm.EXTR_VIS = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osm.EXTR_VIS.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osm.EXTR_TOP = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osm.EXTR_TOP.append(val1)
      _x = self
      start = end
      end += 24
      (_x.osm.EXTR_3D.X, _x.osm.EXTR_3D.Y, _x.osm.EXTR_3D.Z,) = _struct_3d.unpack(str[start:end])
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
      buff.write(_struct_ih.pack(_x.osm.TYPE, _x.osm.ID))
      length = len(self.osm.COM)
      buff.write(_struct_I.pack(length))
      for val1 in self.osm.COM:
        _x = val1
        buff.write(_struct_3d.pack(_x.X, _x.Y, _x.Z))
      length = len(self.osm.EXTR_VIS)
      buff.write(_struct_I.pack(length))
      for val1 in self.osm.EXTR_VIS:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.osm.EXTR_TOP)
      buff.write(_struct_I.pack(length))
      for val1 in self.osm.EXTR_TOP:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      _x = self
      buff.write(_struct_3d.pack(_x.osm.EXTR_3D.X, _x.osm.EXTR_3D.Y, _x.osm.EXTR_3D.Z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.osm is None:
        self.osm = C11_Agent.msg.C24C11_OSM()
      end = 0
      _x = self
      start = end
      end += 6
      (_x.osm.TYPE, _x.osm.ID,) = _struct_ih.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osm.COM = []
      for i in range(0, length):
        val1 = C11_Agent.msg.coord()
        _x = val1
        start = end
        end += 24
        (_x.X, _x.Y, _x.Z,) = _struct_3d.unpack(str[start:end])
        self.osm.COM.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osm.EXTR_VIS = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osm.EXTR_VIS.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.osm.EXTR_TOP = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.osm.EXTR_TOP.append(val1)
      _x = self
      start = end
      end += 24
      (_x.osm.EXTR_3D.X, _x.osm.EXTR_3D.Y, _x.osm.EXTR_3D.Z,) = _struct_3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2d = struct.Struct("<2d")
_struct_ih = struct.Struct("<ih")
_struct_3d = struct.Struct("<3d")
class obstacle_map(object):
  _type          = 'C11_Agent/obstacle_map'
  _md5sum = '9ec3dfc0c61d90cd4fc19bd886162877'
  _request_class  = obstacle_mapRequest
  _response_class = obstacle_mapResponse
