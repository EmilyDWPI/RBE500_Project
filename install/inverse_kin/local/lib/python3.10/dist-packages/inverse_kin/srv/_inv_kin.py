# generated from rosidl_generator_py/resource/_idl.py.em
# with input from inverse_kin:srv/InvKin.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_InvKin_Request(type):
    """Metaclass of message 'InvKin_Request'."""

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
            module = import_type_support('inverse_kin')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'inverse_kin.srv.InvKin_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__inv_kin__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__inv_kin__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__inv_kin__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__inv_kin__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__inv_kin__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InvKin_Request(metaclass=Metaclass_InvKin_Request):
    """Message class 'InvKin_Request'."""

    __slots__ = [
        '_x',
        '_y',
        '_z',
        '_quat_x',
        '_quat_y',
        '_quat_z',
        '_quat_w',
    ]

    _fields_and_field_types = {
        'x': 'double',
        'y': 'double',
        'z': 'double',
        'quat_x': 'double',
        'quat_y': 'double',
        'quat_z': 'double',
        'quat_w': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.quat_x = kwargs.get('quat_x', float())
        self.quat_y = kwargs.get('quat_y', float())
        self.quat_z = kwargs.get('quat_z', float())
        self.quat_w = kwargs.get('quat_w', float())

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.quat_x != other.quat_x:
            return False
        if self.quat_y != other.quat_y:
            return False
        if self.quat_z != other.quat_z:
            return False
        if self.quat_w != other.quat_w:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y = value

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._z = value

    @builtins.property
    def quat_x(self):
        """Message field 'quat_x'."""
        return self._quat_x

    @quat_x.setter
    def quat_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'quat_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'quat_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._quat_x = value

    @builtins.property
    def quat_y(self):
        """Message field 'quat_y'."""
        return self._quat_y

    @quat_y.setter
    def quat_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'quat_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'quat_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._quat_y = value

    @builtins.property
    def quat_z(self):
        """Message field 'quat_z'."""
        return self._quat_z

    @quat_z.setter
    def quat_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'quat_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'quat_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._quat_z = value

    @builtins.property
    def quat_w(self):
        """Message field 'quat_w'."""
        return self._quat_w

    @quat_w.setter
    def quat_w(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'quat_w' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'quat_w' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._quat_w = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_InvKin_Response(type):
    """Metaclass of message 'InvKin_Response'."""

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
            module = import_type_support('inverse_kin')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'inverse_kin.srv.InvKin_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__inv_kin__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__inv_kin__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__inv_kin__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__inv_kin__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__inv_kin__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InvKin_Response(metaclass=Metaclass_InvKin_Response):
    """Message class 'InvKin_Response'."""

    __slots__ = [
        '_q1',
        '_q2',
        '_q3',
    ]

    _fields_and_field_types = {
        'q1': 'double',
        'q2': 'double',
        'q3': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.q1 = kwargs.get('q1', float())
        self.q2 = kwargs.get('q2', float())
        self.q3 = kwargs.get('q3', float())

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
        if self.q1 != other.q1:
            return False
        if self.q2 != other.q2:
            return False
        if self.q3 != other.q3:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def q1(self):
        """Message field 'q1'."""
        return self._q1

    @q1.setter
    def q1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'q1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._q1 = value

    @builtins.property
    def q2(self):
        """Message field 'q2'."""
        return self._q2

    @q2.setter
    def q2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'q2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._q2 = value

    @builtins.property
    def q3(self):
        """Message field 'q3'."""
        return self._q3

    @q3.setter
    def q3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'q3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._q3 = value


class Metaclass_InvKin(type):
    """Metaclass of service 'InvKin'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('inverse_kin')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'inverse_kin.srv.InvKin')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__inv_kin

            from inverse_kin.srv import _inv_kin
            if _inv_kin.Metaclass_InvKin_Request._TYPE_SUPPORT is None:
                _inv_kin.Metaclass_InvKin_Request.__import_type_support__()
            if _inv_kin.Metaclass_InvKin_Response._TYPE_SUPPORT is None:
                _inv_kin.Metaclass_InvKin_Response.__import_type_support__()


class InvKin(metaclass=Metaclass_InvKin):
    from inverse_kin.srv._inv_kin import InvKin_Request as Request
    from inverse_kin.srv._inv_kin import InvKin_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
