# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pplanner_interfaces:msg/ArucoData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ArucoData(type):
    """Metaclass of message 'ArucoData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pplanner_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pplanner_interfaces.msg.ArucoData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__aruco_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__aruco_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__aruco_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__aruco_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__aruco_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ArucoData(metaclass=Metaclass_ArucoData):
    """Message class 'ArucoData'."""

    __slots__ = [
        '_id_data',
        '_x_data',
        '_y_data',
        '_orientation_data',
        '_status',
    ]

    _fields_and_field_types = {
        'id_data': 'int64',
        'x_data': 'int64',
        'y_data': 'int64',
        'orientation_data': 'double',
        'status': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id_data = kwargs.get('id_data', int())
        self.x_data = kwargs.get('x_data', int())
        self.y_data = kwargs.get('y_data', int())
        self.orientation_data = kwargs.get('orientation_data', float())
        self.status = kwargs.get('status', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.id_data != other.id_data:
            return False
        if self.x_data != other.x_data:
            return False
        if self.y_data != other.y_data:
            return False
        if self.orientation_data != other.orientation_data:
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def id_data(self):
        """Message field 'id_data'."""
        return self._id_data

    @id_data.setter
    def id_data(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id_data' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'id_data' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._id_data = value

    @builtins.property
    def x_data(self):
        """Message field 'x_data'."""
        return self._x_data

    @x_data.setter
    def x_data(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'x_data' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'x_data' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._x_data = value

    @builtins.property
    def y_data(self):
        """Message field 'y_data'."""
        return self._y_data

    @y_data.setter
    def y_data(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'y_data' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'y_data' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._y_data = value

    @builtins.property
    def orientation_data(self):
        """Message field 'orientation_data'."""
        return self._orientation_data

    @orientation_data.setter
    def orientation_data(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'orientation_data' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'orientation_data' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._orientation_data = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status' field must be of type 'str'"
        self._status = value
