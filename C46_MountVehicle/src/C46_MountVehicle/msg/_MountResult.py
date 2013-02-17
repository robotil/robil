"""autogenerated by genpy from C46_MountVehicle/MountResult.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


<<<<<<< HEAD:C46_MountVehicle/src/C46_MountVehicle/msg/_MountResult.py
class MountResult(genpy.Message):
  _md5sum = "8e265b97fe86999f6bcce88ace33a325"
  _type = "C46_MountVehicle/MountResult"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Define the result
float32 mounted

"""
  __slots__ = ['mounted']
  _slot_types = ['float32']
=======
class PostureControlFeedback(genpy.Message):
  _md5sum = "25141b31986ff139053dd8744ab38294"
  _type = "C45_PostureControl/PostureControlFeedback"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#feedback
float64 stabilityQuality

"""
  __slots__ = ['stabilityQuality']
  _slot_types = ['float64']
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c:C45_PostureControl/src/C45_PostureControl/msg/_PostureControlFeedback.py

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
<<<<<<< HEAD:C46_MountVehicle/src/C46_MountVehicle/msg/_MountResult.py
       mounted
=======
       stabilityQuality
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c:C45_PostureControl/src/C45_PostureControl/msg/_PostureControlFeedback.py

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MountResult, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
<<<<<<< HEAD:C46_MountVehicle/src/C46_MountVehicle/msg/_MountResult.py
      if self.mounted is None:
        self.mounted = 0.
    else:
      self.mounted = 0.
=======
      if self.stabilityQuality is None:
        self.stabilityQuality = 0.
    else:
      self.stabilityQuality = 0.
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c:C45_PostureControl/src/C45_PostureControl/msg/_PostureControlFeedback.py

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
<<<<<<< HEAD:C46_MountVehicle/src/C46_MountVehicle/msg/_MountResult.py
      buff.write(_struct_f.pack(self.mounted))
=======
      buff.write(_struct_d.pack(self.stabilityQuality))
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c:C45_PostureControl/src/C45_PostureControl/msg/_PostureControlFeedback.py
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
<<<<<<< HEAD:C46_MountVehicle/src/C46_MountVehicle/msg/_MountResult.py
      end += 4
      (self.mounted,) = _struct_f.unpack(str[start:end])
=======
      end += 8
      (self.stabilityQuality,) = _struct_d.unpack(str[start:end])
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c:C45_PostureControl/src/C45_PostureControl/msg/_PostureControlFeedback.py
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
<<<<<<< HEAD:C46_MountVehicle/src/C46_MountVehicle/msg/_MountResult.py
      buff.write(_struct_f.pack(self.mounted))
=======
      buff.write(_struct_d.pack(self.stabilityQuality))
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c:C45_PostureControl/src/C45_PostureControl/msg/_PostureControlFeedback.py
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
<<<<<<< HEAD:C46_MountVehicle/src/C46_MountVehicle/msg/_MountResult.py
      end += 4
      (self.mounted,) = _struct_f.unpack(str[start:end])
=======
      end += 8
      (self.stabilityQuality,) = _struct_d.unpack(str[start:end])
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c:C45_PostureControl/src/C45_PostureControl/msg/_PostureControlFeedback.py
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
<<<<<<< HEAD:C46_MountVehicle/src/C46_MountVehicle/msg/_MountResult.py
_struct_f = struct.Struct("<f")
=======
_struct_d = struct.Struct("<d")
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c:C45_PostureControl/src/C45_PostureControl/msg/_PostureControlFeedback.py