"""autogenerated by genpy from C11_Agent/C11C23_OBM.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class C11C23_OBM(genpy.Message):
  _md5sum = "021b2b252d90f9fb11b61af4d8bd8a44"
  _type = "C11_Agent/C11C23_OBM"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 LAT
int16 SCN
int16 SCN_SCAN=0
int16 SCN_CURRENT=1
int16 MOV
int16 MOV_NONE     = 0
int16 MOV_HEAD     = 1
int16 MOV_POSTURE  = 2
int16 MOV_POSITION = 3
"""
  # Pseudo-constants
  SCN_SCAN = 0
  SCN_CURRENT = 1
  MOV_NONE = 0
  MOV_HEAD = 1
  MOV_POSTURE = 2
  MOV_POSITION = 3

  __slots__ = ['LAT','SCN','MOV']
  _slot_types = ['float64','int16','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       LAT,SCN,MOV

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(C11C23_OBM, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.LAT is None:
        self.LAT = 0.
      if self.SCN is None:
        self.SCN = 0
      if self.MOV is None:
        self.MOV = 0
    else:
      self.LAT = 0.
      self.SCN = 0
      self.MOV = 0

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
      buff.write(_struct_d2h.pack(_x.LAT, _x.SCN, _x.MOV))
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
      end += 12
      (_x.LAT, _x.SCN, _x.MOV,) = _struct_d2h.unpack(str[start:end])
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
      buff.write(_struct_d2h.pack(_x.LAT, _x.SCN, _x.MOV))
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
      end += 12
      (_x.LAT, _x.SCN, _x.MOV,) = _struct_d2h.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_d2h = struct.Struct("<d2h")
