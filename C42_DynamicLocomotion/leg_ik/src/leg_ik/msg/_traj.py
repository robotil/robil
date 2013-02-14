"""autogenerated by genpy from leg_ik/traj.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class traj(genpy.Message):
  _md5sum = "5ec2ca23b0e126f2f1c1263a9a369c4b"
  _type = "leg_ik/traj"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 COMx
float64 COMy
float64 COMz
float64 Swing_x
float64 Swing_y
float64 Swing_z
int32   leg

"""
  __slots__ = ['COMx','COMy','COMz','Swing_x','Swing_y','Swing_z','leg']
  _slot_types = ['float64','float64','float64','float64','float64','float64','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       COMx,COMy,COMz,Swing_x,Swing_y,Swing_z,leg

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(traj, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.COMx is None:
        self.COMx = 0.
      if self.COMy is None:
        self.COMy = 0.
      if self.COMz is None:
        self.COMz = 0.
      if self.Swing_x is None:
        self.Swing_x = 0.
      if self.Swing_y is None:
        self.Swing_y = 0.
      if self.Swing_z is None:
        self.Swing_z = 0.
      if self.leg is None:
        self.leg = 0
    else:
      self.COMx = 0.
      self.COMy = 0.
      self.COMz = 0.
      self.Swing_x = 0.
      self.Swing_y = 0.
      self.Swing_z = 0.
      self.leg = 0

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
      buff.write(_struct_6di.pack(_x.COMx, _x.COMy, _x.COMz, _x.Swing_x, _x.Swing_y, _x.Swing_z, _x.leg))
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
      end += 52
      (_x.COMx, _x.COMy, _x.COMz, _x.Swing_x, _x.Swing_y, _x.Swing_z, _x.leg,) = _struct_6di.unpack(str[start:end])
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
      buff.write(_struct_6di.pack(_x.COMx, _x.COMy, _x.COMz, _x.Swing_x, _x.Swing_y, _x.Swing_z, _x.leg))
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
      end += 52
      (_x.COMx, _x.COMy, _x.COMz, _x.Swing_x, _x.Swing_y, _x.Swing_z, _x.leg,) = _struct_6di.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6di = struct.Struct("<6di")
