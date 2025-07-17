# generated from rosidl_generator_py/resource/_idl.py.em
# with input from inverse_kin:msg/InvKin.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_InvKin(type):
    """Metaclass of message 'InvKin'."""

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
                'inverse_kin.msg.InvKin')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__inv_kin
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__inv_kin
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__inv_kin
            cls._TYPE_SUPPORT = module.type_support_msg__msg__inv_kin
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__inv_kin

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InvKin(metaclass=Metaclass_InvKin):
    """Message class 'InvKin'."""

    __slots__ = [
        '_scara_pose',
        '_ee_pos',
        '_ee_angl',
    ]

    _fields_and_field_types = {
        'scara_pose': 'geometry_msgs/Pose',
        'ee_pos': 'geometry_msgs/Point',
        'ee_angl': 'geometry_msgs/Quaternion',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Pose
        self.scara_pose = kwargs.get('scara_pose', Pose())
        from geometry_msgs.msg import Point
        self.ee_pos = kwargs.get('ee_pos', Point())
        from geometry_msgs.msg import Quaternion
        self.ee_angl = kwargs.get('ee_angl', Quaternion())

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
        if self.scara_pose != other.scara_pose:
            return False
        if self.ee_pos != other.ee_pos:
            return False
        if self.ee_angl != other.ee_angl:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def scara_pose(self):
        """Message field 'scara_pose'."""
        return self._scara_pose

    @scara_pose.setter
    def scara_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'scara_pose' field must be a sub message of type 'Pose'"
        self._scara_pose = value

    @builtins.property
    def ee_pos(self):
        """Message field 'ee_pos'."""
        return self._ee_pos

    @ee_pos.setter
    def ee_pos(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'ee_pos' field must be a sub message of type 'Point'"
        self._ee_pos = value

    @builtins.property
    def ee_angl(self):
        """Message field 'ee_angl'."""
        return self._ee_angl

    @ee_angl.setter
    def ee_angl(self, value):
        if __debug__:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'ee_angl' field must be a sub message of type 'Quaternion'"
        self._ee_angl = value
