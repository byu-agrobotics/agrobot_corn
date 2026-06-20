# generated from rosidl_generator_py/resource/_idl.py.em
# with input from basicmicro_ros2:srv/SetPositionLimits.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetPositionLimits_Request(type):
    """Metaclass of message 'SetPositionLimits_Request'."""

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
            module = import_type_support('basicmicro_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'basicmicro_ros2.srv.SetPositionLimits_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_position_limits__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_position_limits__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_position_limits__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_position_limits__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_position_limits__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetPositionLimits_Request(metaclass=Metaclass_SetPositionLimits_Request):
    """Message class 'SetPositionLimits_Request'."""

    __slots__ = [
        '_enable_limits',
        '_left_min_position',
        '_left_max_position',
        '_right_min_position',
        '_right_max_position',
        '_violation_behavior',
        '_decel_rate',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'enable_limits': 'boolean',
        'left_min_position': 'double',
        'left_max_position': 'double',
        'right_min_position': 'double',
        'right_max_position': 'double',
        'violation_behavior': 'string',
        'decel_rate': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.enable_limits = kwargs.get('enable_limits', bool())
        self.left_min_position = kwargs.get('left_min_position', float())
        self.left_max_position = kwargs.get('left_max_position', float())
        self.right_min_position = kwargs.get('right_min_position', float())
        self.right_max_position = kwargs.get('right_max_position', float())
        self.violation_behavior = kwargs.get('violation_behavior', str())
        self.decel_rate = kwargs.get('decel_rate', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.enable_limits != other.enable_limits:
            return False
        if self.left_min_position != other.left_min_position:
            return False
        if self.left_max_position != other.left_max_position:
            return False
        if self.right_min_position != other.right_min_position:
            return False
        if self.right_max_position != other.right_max_position:
            return False
        if self.violation_behavior != other.violation_behavior:
            return False
        if self.decel_rate != other.decel_rate:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def enable_limits(self):
        """Message field 'enable_limits'."""
        return self._enable_limits

    @enable_limits.setter
    def enable_limits(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'enable_limits' field must be of type 'bool'"
        self._enable_limits = value

    @builtins.property
    def left_min_position(self):
        """Message field 'left_min_position'."""
        return self._left_min_position

    @left_min_position.setter
    def left_min_position(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'left_min_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_min_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_min_position = value

    @builtins.property
    def left_max_position(self):
        """Message field 'left_max_position'."""
        return self._left_max_position

    @left_max_position.setter
    def left_max_position(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'left_max_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'left_max_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._left_max_position = value

    @builtins.property
    def right_min_position(self):
        """Message field 'right_min_position'."""
        return self._right_min_position

    @right_min_position.setter
    def right_min_position(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'right_min_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_min_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_min_position = value

    @builtins.property
    def right_max_position(self):
        """Message field 'right_max_position'."""
        return self._right_max_position

    @right_max_position.setter
    def right_max_position(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'right_max_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'right_max_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._right_max_position = value

    @builtins.property
    def violation_behavior(self):
        """Message field 'violation_behavior'."""
        return self._violation_behavior

    @violation_behavior.setter
    def violation_behavior(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'violation_behavior' field must be of type 'str'"
        self._violation_behavior = value

    @builtins.property
    def decel_rate(self):
        """Message field 'decel_rate'."""
        return self._decel_rate

    @decel_rate.setter
    def decel_rate(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'decel_rate' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'decel_rate' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._decel_rate = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetPositionLimits_Response(type):
    """Metaclass of message 'SetPositionLimits_Response'."""

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
            module = import_type_support('basicmicro_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'basicmicro_ros2.srv.SetPositionLimits_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_position_limits__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_position_limits__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_position_limits__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_position_limits__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_position_limits__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetPositionLimits_Response(metaclass=Metaclass_SetPositionLimits_Response):
    """Message class 'SetPositionLimits_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetPositionLimits_Event(type):
    """Metaclass of message 'SetPositionLimits_Event'."""

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
            module = import_type_support('basicmicro_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'basicmicro_ros2.srv.SetPositionLimits_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_position_limits__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_position_limits__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_position_limits__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_position_limits__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_position_limits__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetPositionLimits_Event(metaclass=Metaclass_SetPositionLimits_Event):
    """Message class 'SetPositionLimits_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<basicmicro_ros2/SetPositionLimits_Request, 1>',
        'response': 'sequence<basicmicro_ros2/SetPositionLimits_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['basicmicro_ros2', 'srv'], 'SetPositionLimits_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['basicmicro_ros2', 'srv'], 'SetPositionLimits_Response'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from basicmicro_ros2.srv import SetPositionLimits_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, SetPositionLimits_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'SetPositionLimits_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from basicmicro_ros2.srv import SetPositionLimits_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, SetPositionLimits_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'SetPositionLimits_Response'"
        self._response = value


class Metaclass_SetPositionLimits(type):
    """Metaclass of service 'SetPositionLimits'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('basicmicro_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'basicmicro_ros2.srv.SetPositionLimits')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_position_limits

            from basicmicro_ros2.srv import _set_position_limits
            if _set_position_limits.Metaclass_SetPositionLimits_Request._TYPE_SUPPORT is None:
                _set_position_limits.Metaclass_SetPositionLimits_Request.__import_type_support__()
            if _set_position_limits.Metaclass_SetPositionLimits_Response._TYPE_SUPPORT is None:
                _set_position_limits.Metaclass_SetPositionLimits_Response.__import_type_support__()
            if _set_position_limits.Metaclass_SetPositionLimits_Event._TYPE_SUPPORT is None:
                _set_position_limits.Metaclass_SetPositionLimits_Event.__import_type_support__()


class SetPositionLimits(metaclass=Metaclass_SetPositionLimits):
    from basicmicro_ros2.srv._set_position_limits import SetPositionLimits_Request as Request
    from basicmicro_ros2.srv._set_position_limits import SetPositionLimits_Response as Response
    from basicmicro_ros2.srv._set_position_limits import SetPositionLimits_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
