# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from sensor_msgs/BatteryState.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import rosserial.genpy as genpy
import struct

import rosserial.std_msgs.msg

class BatteryState(genpy.Message):
  _md5sum = "476f837fa6771f6e16e3bf4ef96f8770"
  _type = "sensor_msgs/BatteryState"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """
# Constants are chosen to match the enums in the linux kernel
# defined in include/linux/power_supply.h as of version 3.7
# The one difference is for style reasons the constants are
# all uppercase not mixed case.

# Power supply status constants
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4

# Power supply health constants
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

# Power supply technology (chemistry) constants
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

Header  header
float32 voltage          # Voltage in Volts (Mandatory)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 charge           # Current charge in Ah  (If unmeasured NaN)
float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
uint8   power_supply_technology # The battery chemistry. Values defined above
bool    present          # True if the battery is present

float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                         # If individual voltages unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  # Pseudo-constants
  POWER_SUPPLY_STATUS_UNKNOWN = 0
  POWER_SUPPLY_STATUS_CHARGING = 1
  POWER_SUPPLY_STATUS_DISCHARGING = 2
  POWER_SUPPLY_STATUS_NOT_CHARGING = 3
  POWER_SUPPLY_STATUS_FULL = 4
  POWER_SUPPLY_HEALTH_UNKNOWN = 0
  POWER_SUPPLY_HEALTH_GOOD = 1
  POWER_SUPPLY_HEALTH_OVERHEAT = 2
  POWER_SUPPLY_HEALTH_DEAD = 3
  POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
  POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
  POWER_SUPPLY_HEALTH_COLD = 6
  POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
  POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8
  POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
  POWER_SUPPLY_TECHNOLOGY_NIMH = 1
  POWER_SUPPLY_TECHNOLOGY_LION = 2
  POWER_SUPPLY_TECHNOLOGY_LIPO = 3
  POWER_SUPPLY_TECHNOLOGY_LIFE = 4
  POWER_SUPPLY_TECHNOLOGY_NICD = 5
  POWER_SUPPLY_TECHNOLOGY_LIMN = 6

  __slots__ = ['header','voltage','current','charge','capacity','design_capacity','percentage','power_supply_status','power_supply_health','power_supply_technology','present','cell_voltage','location','serial_number']
  _slot_types = ['std_msgs/Header','float32','float32','float32','float32','float32','float32','uint8','uint8','uint8','bool','float32[]','string','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,voltage,current,charge,capacity,design_capacity,percentage,power_supply_status,power_supply_health,power_supply_technology,present,cell_voltage,location,serial_number

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(BatteryState, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = rosserial.rosserial.std_msgs.msg.Header()
      if self.voltage is None:
        self.voltage = 0.
      if self.current is None:
        self.current = 0.
      if self.charge is None:
        self.charge = 0.
      if self.capacity is None:
        self.capacity = 0.
      if self.design_capacity is None:
        self.design_capacity = 0.
      if self.percentage is None:
        self.percentage = 0.
      if self.power_supply_status is None:
        self.power_supply_status = 0
      if self.power_supply_health is None:
        self.power_supply_health = 0
      if self.power_supply_technology is None:
        self.power_supply_technology = 0
      if self.present is None:
        self.present = False
      if self.cell_voltage is None:
        self.cell_voltage = []
      if self.location is None:
        self.location = ''
      if self.serial_number is None:
        self.serial_number = ''
    else:
      self.header = rosserial.rosserial.std_msgs.msg.Header()
      self.voltage = 0.
      self.current = 0.
      self.charge = 0.
      self.capacity = 0.
      self.design_capacity = 0.
      self.percentage = 0.
      self.power_supply_status = 0
      self.power_supply_health = 0
      self.power_supply_technology = 0
      self.present = False
      self.cell_voltage = []
      self.location = ''
      self.serial_number = ''

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_6f4B().pack(_x.voltage, _x.current, _x.charge, _x.capacity, _x.design_capacity, _x.percentage, _x.power_supply_status, _x.power_supply_health, _x.power_supply_technology, _x.present))
      length = len(self.cell_voltage)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.cell_voltage))
      _x = self.location
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.serial_number
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = rosserial.rosserial.std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 28
      (_x.voltage, _x.current, _x.charge, _x.capacity, _x.design_capacity, _x.percentage, _x.power_supply_status, _x.power_supply_health, _x.power_supply_technology, _x.present,) = _get_struct_6f4B().unpack(str[start:end])
      self.present = bool(self.present)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.cell_voltage = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.location = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.location = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.serial_number = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.serial_number = str[start:end]
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_6f4B().pack(_x.voltage, _x.current, _x.charge, _x.capacity, _x.design_capacity, _x.percentage, _x.power_supply_status, _x.power_supply_health, _x.power_supply_technology, _x.present))
      length = len(self.cell_voltage)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.cell_voltage.tostring())
      _x = self.location
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.serial_number
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = rosserial.rosserial.std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 28
      (_x.voltage, _x.current, _x.charge, _x.capacity, _x.design_capacity, _x.percentage, _x.power_supply_status, _x.power_supply_health, _x.power_supply_technology, _x.present,) = _get_struct_6f4B().unpack(str[start:end])
      self.present = bool(self.present)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.cell_voltage = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.location = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.location = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.serial_number = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.serial_number = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_6f4B = None
def _get_struct_6f4B():
    global _struct_6f4B
    if _struct_6f4B is None:
        _struct_6f4B = struct.Struct("<6f4B")
    return _struct_6f4B
