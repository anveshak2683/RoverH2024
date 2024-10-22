# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from navigation2/auto.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class auto(genpy.Message):
  _md5sum = "65672b1bd9673047ebc0672e3417ec0f"
  _type = "navigation2/auto"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool arm
float64 latitude 
float64 longitude
int8 setstage 
string text
float64[] aruco_coordinates
string reference 
int8 msg_id

"""
  __slots__ = ['arm','latitude','longitude','setstage','text','aruco_coordinates','reference','msg_id']
  _slot_types = ['bool','float64','float64','int8','string','float64[]','string','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       arm,latitude,longitude,setstage,text,aruco_coordinates,reference,msg_id

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(auto, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.arm is None:
        self.arm = False
      if self.latitude is None:
        self.latitude = 0.
      if self.longitude is None:
        self.longitude = 0.
      if self.setstage is None:
        self.setstage = 0
      if self.text is None:
        self.text = ''
      if self.aruco_coordinates is None:
        self.aruco_coordinates = []
      if self.reference is None:
        self.reference = ''
      if self.msg_id is None:
        self.msg_id = 0
    else:
      self.arm = False
      self.latitude = 0.
      self.longitude = 0.
      self.setstage = 0
      self.text = ''
      self.aruco_coordinates = []
      self.reference = ''
      self.msg_id = 0

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
      buff.write(_get_struct_B2db().pack(_x.arm, _x.latitude, _x.longitude, _x.setstage))
      _x = self.text
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.aruco_coordinates)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.aruco_coordinates))
      _x = self.reference
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.msg_id
      buff.write(_get_struct_b().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 18
      (_x.arm, _x.latitude, _x.longitude, _x.setstage,) = _get_struct_B2db().unpack(str[start:end])
      self.arm = bool(self.arm)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.text = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.text = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.aruco_coordinates = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.reference = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.reference = str[start:end]
      start = end
      end += 1
      (self.msg_id,) = _get_struct_b().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_B2db().pack(_x.arm, _x.latitude, _x.longitude, _x.setstage))
      _x = self.text
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.aruco_coordinates)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.aruco_coordinates.tostring())
      _x = self.reference
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.msg_id
      buff.write(_get_struct_b().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 18
      (_x.arm, _x.latitude, _x.longitude, _x.setstage,) = _get_struct_B2db().unpack(str[start:end])
      self.arm = bool(self.arm)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.text = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.text = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.aruco_coordinates = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.reference = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.reference = str[start:end]
      start = end
      end += 1
      (self.msg_id,) = _get_struct_b().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B2db = None
def _get_struct_B2db():
    global _struct_B2db
    if _struct_B2db is None:
        _struct_B2db = struct.Struct("<B2db")
    return _struct_B2db
_struct_b = None
def _get_struct_b():
    global _struct_b
    if _struct_b is None:
        _struct_b = struct.Struct("<b")
    return _struct_b
