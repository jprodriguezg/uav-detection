"""autogenerated by genpy from drone_GPS/GPS_data.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class GPS_data(genpy.Message):
  _md5sum = "66dd7373cf0e63f95829d59e518dc508"
  _type = "drone_GPS/GPS_data"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# header
Header      header
# ----------------- Data GPS ----------------------------
float64     longitude
float64     latitude
float64     elevation
float64     X
float64     Y
int32     time_zone
int32       altitude
uint8 num_sattelites



================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','longitude','latitude','elevation','X','Y','time_zone','altitude','num_sattelites']
  _slot_types = ['std_msgs/Header','float64','float64','float64','float64','float64','int32','int32','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,longitude,latitude,elevation,X,Y,time_zone,altitude,num_sattelites

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GPS_data, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.longitude is None:
        self.longitude = 0.
      if self.latitude is None:
        self.latitude = 0.
      if self.elevation is None:
        self.elevation = 0.
      if self.X is None:
        self.X = 0.
      if self.Y is None:
        self.Y = 0.
      if self.time_zone is None:
        self.time_zone = 0
      if self.altitude is None:
        self.altitude = 0
      if self.num_sattelites is None:
        self.num_sattelites = 0
    else:
      self.header = std_msgs.msg.Header()
      self.longitude = 0.
      self.latitude = 0.
      self.elevation = 0.
      self.X = 0.
      self.Y = 0.
      self.time_zone = 0
      self.altitude = 0
      self.num_sattelites = 0

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_5d2iB.pack(_x.longitude, _x.latitude, _x.elevation, _x.X, _x.Y, _x.time_zone, _x.altitude, _x.num_sattelites))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 49
      (_x.longitude, _x.latitude, _x.elevation, _x.X, _x.Y, _x.time_zone, _x.altitude, _x.num_sattelites,) = _struct_5d2iB.unpack(str[start:end])
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_5d2iB.pack(_x.longitude, _x.latitude, _x.elevation, _x.X, _x.Y, _x.time_zone, _x.altitude, _x.num_sattelites))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 49
      (_x.longitude, _x.latitude, _x.elevation, _x.X, _x.Y, _x.time_zone, _x.altitude, _x.num_sattelites,) = _struct_5d2iB.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_5d2iB = struct.Struct("<5d2iB")
