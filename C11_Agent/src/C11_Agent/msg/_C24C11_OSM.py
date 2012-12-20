"""autogenerated by genpy from C11_Agent/C24C11_OSM.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import C11_Agent.msg

class C24C11_OSM(genpy.Message):
  _md5sum = "9bfebbc3c0618ea996554dd91c71e678"
  _type = "C11_Agent/C24C11_OSM"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 TYPE
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
  # Pseudo-constants
  TYPE_STATIC = 0
  TYPE_DYNAMIC = 1

  __slots__ = ['TYPE','ID','COM','EXTR_VIS','EXTR_TOP','EXTR_3D']
  _slot_types = ['int32','int16','C11_Agent/coord[]','C11_Agent/pathLocation[]','C11_Agent/pathLocation[]','C11_Agent/coord']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       TYPE,ID,COM,EXTR_VIS,EXTR_TOP,EXTR_3D

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(C24C11_OSM, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.TYPE is None:
        self.TYPE = 0
      if self.ID is None:
        self.ID = 0
      if self.COM is None:
        self.COM = []
      if self.EXTR_VIS is None:
        self.EXTR_VIS = []
      if self.EXTR_TOP is None:
        self.EXTR_TOP = []
      if self.EXTR_3D is None:
        self.EXTR_3D = C11_Agent.msg.coord()
    else:
      self.TYPE = 0
      self.ID = 0
      self.COM = []
      self.EXTR_VIS = []
      self.EXTR_TOP = []
      self.EXTR_3D = C11_Agent.msg.coord()

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
      buff.write(_struct_ih.pack(_x.TYPE, _x.ID))
      length = len(self.COM)
      buff.write(_struct_I.pack(length))
      for val1 in self.COM:
        _x = val1
        buff.write(_struct_3d.pack(_x.X, _x.Y, _x.Z))
      length = len(self.EXTR_VIS)
      buff.write(_struct_I.pack(length))
      for val1 in self.EXTR_VIS:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.EXTR_TOP)
      buff.write(_struct_I.pack(length))
      for val1 in self.EXTR_TOP:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      _x = self
      buff.write(_struct_3d.pack(_x.EXTR_3D.X, _x.EXTR_3D.Y, _x.EXTR_3D.Z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.COM is None:
        self.COM = None
      if self.EXTR_VIS is None:
        self.EXTR_VIS = None
      if self.EXTR_TOP is None:
        self.EXTR_TOP = None
      if self.EXTR_3D is None:
        self.EXTR_3D = C11_Agent.msg.coord()
      end = 0
      _x = self
      start = end
      end += 6
      (_x.TYPE, _x.ID,) = _struct_ih.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.COM = []
      for i in range(0, length):
        val1 = C11_Agent.msg.coord()
        _x = val1
        start = end
        end += 24
        (_x.X, _x.Y, _x.Z,) = _struct_3d.unpack(str[start:end])
        self.COM.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.EXTR_VIS = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.EXTR_VIS.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.EXTR_TOP = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.EXTR_TOP.append(val1)
      _x = self
      start = end
      end += 24
      (_x.EXTR_3D.X, _x.EXTR_3D.Y, _x.EXTR_3D.Z,) = _struct_3d.unpack(str[start:end])
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
      buff.write(_struct_ih.pack(_x.TYPE, _x.ID))
      length = len(self.COM)
      buff.write(_struct_I.pack(length))
      for val1 in self.COM:
        _x = val1
        buff.write(_struct_3d.pack(_x.X, _x.Y, _x.Z))
      length = len(self.EXTR_VIS)
      buff.write(_struct_I.pack(length))
      for val1 in self.EXTR_VIS:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      length = len(self.EXTR_TOP)
      buff.write(_struct_I.pack(length))
      for val1 in self.EXTR_TOP:
        _x = val1
        buff.write(_struct_2d.pack(_x.lat, _x.lon))
      _x = self
      buff.write(_struct_3d.pack(_x.EXTR_3D.X, _x.EXTR_3D.Y, _x.EXTR_3D.Z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.COM is None:
        self.COM = None
      if self.EXTR_VIS is None:
        self.EXTR_VIS = None
      if self.EXTR_TOP is None:
        self.EXTR_TOP = None
      if self.EXTR_3D is None:
        self.EXTR_3D = C11_Agent.msg.coord()
      end = 0
      _x = self
      start = end
      end += 6
      (_x.TYPE, _x.ID,) = _struct_ih.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.COM = []
      for i in range(0, length):
        val1 = C11_Agent.msg.coord()
        _x = val1
        start = end
        end += 24
        (_x.X, _x.Y, _x.Z,) = _struct_3d.unpack(str[start:end])
        self.COM.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.EXTR_VIS = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.EXTR_VIS.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.EXTR_TOP = []
      for i in range(0, length):
        val1 = C11_Agent.msg.pathLocation()
        _x = val1
        start = end
        end += 16
        (_x.lat, _x.lon,) = _struct_2d.unpack(str[start:end])
        self.EXTR_TOP.append(val1)
      _x = self
      start = end
      end += 24
      (_x.EXTR_3D.X, _x.EXTR_3D.Y, _x.EXTR_3D.Z,) = _struct_3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2d = struct.Struct("<2d")
_struct_ih = struct.Struct("<ih")
_struct_3d = struct.Struct("<3d")
