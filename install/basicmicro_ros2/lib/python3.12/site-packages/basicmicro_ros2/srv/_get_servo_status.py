# generated from rosidl_generator_py/resource/_idl.py.em
# with input from basicmicro_ros2:srv/GetServoStatus.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetServoStatus_Request(type):
    """Metaclass of message 'GetServoStatus_Request'."""

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
                'basicmicro_ros2.srv.GetServoStatus_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_servo_status__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_servo_status__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_servo_status__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_servo_status__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_servo_status__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetServoStatus_Request(metaclass=Metaclass_GetServoStatus_Request):
    """Message class 'GetServoStatus_Request'."""

    __slots__ = [
        '_check_fields',
    ]

    _fields_and_field_types = {
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetServoStatus_Response(type):
    """Metaclass of message 'GetServoStatus_Response'."""

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
                'basicmicro_ros2.srv.GetServoStatus_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_servo_status__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_servo_status__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_servo_status__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_servo_status__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_servo_status__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetServoStatus_Response(metaclass=Metaclass_GetServoStatus_Response):
    """Message class 'GetServoStatus_Response'."""

    __slots__ = [
        '_success',
        '_left_position_error',
        '_right_position_error',
        '_left_speed_error',
        '_right_speed_error',
        '_error_limits_exceeded',
        '_message',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'left_position_error': 'int32',
        'right_position_error': 'int32',
        'left_speed_error': 'int32',
        'right_speed_error': 'int32',
        'error_limits_exceeded': 'boolean',
        'message': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
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
        self.left_position_error = kwargs.get('left_position_error', int())
        self.right_position_error = kwargs.get('right_position_error', int())
        self.left_speed_error = kwargs.get('left_speed_error', int())
        self.right_speed_error = kwargs.get('right_speed_error', int())
        self.error_limits_exceeded = kwargs.get('error_limits_exceeded', bool())
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
        if self.left_position_error != other.left_position_error:
            return False
        if self.right_position_error != other.right_position_error:
            return False
        if self.left_speed_error != other.left_speed_error:
            return False
        if self.right_speed_error != other.right_speed_error:
            return False
        if self.error_limits_exceeded != other.error_limits_exceeded:
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
    def left_position_error(self):
        """Message field 'left_position_error'."""
        return self._left_position_error

    @left_position_error.setter
    def left_position_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'left_position_error' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_position_error' field must be an integer in [-2147483648, 2147483647]"
        self._left_position_error = value

    @builtins.property
    def right_position_error(self):
        """Message field 'right_position_error'."""
        return self._right_position_error

    @right_position_error.setter
    def right_position_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'right_position_error' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_position_error' field must be an integer in [-2147483648, 2147483647]"
        self._right_position_error = value

    @builtins.property
    def left_speed_error(self):
        """Message field 'left_speed_error'."""
        return self._left_speed_error

    @left_speed_error.setter
    def left_speed_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'left_speed_error' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_speed_error' field must be an integer in [-2147483648, 2147483647]"
        self._left_speed_error = value

    @builtins.property
    def right_speed_error(self):
        """Message field 'right_speed_error'."""
        return self._right_speed_error

    @right_speed_error.setter
    def right_speed_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'right_speed_error' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_speed_error' field must be an integer in [-2147483648, 2147483647]"
        self._right_speed_error = value

    @builtins.property
    def error_limits_exceeded(self):
        """Message field 'error_limits_exceeded'."""
        return self._error_limits_exceeded

    @error_limits_exceeded.setter
    def error_limits_exceeded(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'error_limits_exceeded' field must be of type 'bool'"
        self._error_limits_exceeded = value

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


class Metaclass_GetServoStatus_Event(type):
    """Metaclass of message 'GetServoStatus_Event'."""

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
                'basicmicro_ros2.srv.GetServoStatus_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_servo_status__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_servo_status__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_servo_status__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_servo_status__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_servo_status__event

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


class GetServoStatus_Event(metaclass=Metaclass_GetServoStatus_Event):
    """Message class 'GetServoStatus_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<basicmicro_ros2/GetServoStatus_Request, 1>',
        'response': 'sequence<basicmicro_ros2/GetServoStatus_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['basicmicro_ros2', 'srv'], 'GetServoStatus_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['basicmicro_ros2', 'srv'], 'GetServoStatus_Response'), 1),  # noqa: E501
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
            from basicmicro_ros2.srv import GetServoStatus_Request
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
                 all(isinstance(v, GetServoStatus_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'GetServoStatus_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from basicmicro_ros2.srv import GetServoStatus_Response
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
                 all(isinstance(v, GetServoStatus_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'GetServoStatus_Response'"
        self._response = value


class Metaclass_GetServoStatus(type):
    """Metaclass of service 'GetServoStatus'."""

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
                'basicmicro_ros2.srv.GetServoStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_servo_status

            from basicmicro_ros2.srv import _get_servo_status
            if _get_servo_status.Metaclass_GetServoStatus_Request._TYPE_SUPPORT is None:
                _get_servo_status.Metaclass_GetServoStatus_Request.__import_type_support__()
            if _get_servo_status.Metaclass_GetServoStatus_Response._TYPE_SUPPORT is None:
                _get_servo_status.Metaclass_GetServoStatus_Response.__import_type_support__()
            if _get_servo_status.Metaclass_GetServoStatus_Event._TYPE_SUPPORT is None:
                _get_servo_status.Metaclass_GetServoStatus_Event.__import_type_support__()


class GetServoStatus(metaclass=Metaclass_GetServoStatus):
    from basicmicro_ros2.srv._get_servo_status import GetServoStatus_Request as Request
    from basicmicro_ros2.srv._get_servo_status import GetServoStatus_Response as Response
    from basicmicro_ros2.srv._get_servo_status import GetServoStatus_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
