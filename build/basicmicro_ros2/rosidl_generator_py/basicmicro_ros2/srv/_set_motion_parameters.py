# generated from rosidl_generator_py/resource/_idl.py.em
# with input from basicmicro_ros2:srv/SetMotionParameters.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetMotionParameters_Request(type):
    """Metaclass of message 'SetMotionParameters_Request'."""

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
                'basicmicro_ros2.srv.SetMotionParameters_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_motion_parameters__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_motion_parameters__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_motion_parameters__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_motion_parameters__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_motion_parameters__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetMotionParameters_Request(metaclass=Metaclass_SetMotionParameters_Request):
    """Message class 'SetMotionParameters_Request'."""

    __slots__ = [
        '_default_acceleration',
        '_max_speed',
        '_buffer_depth',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'default_acceleration': 'int32',
        'max_speed': 'int32',
        'buffer_depth': 'int32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
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
        self.default_acceleration = kwargs.get('default_acceleration', int())
        self.max_speed = kwargs.get('max_speed', int())
        self.buffer_depth = kwargs.get('buffer_depth', int())

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
        if self.default_acceleration != other.default_acceleration:
            return False
        if self.max_speed != other.max_speed:
            return False
        if self.buffer_depth != other.buffer_depth:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def default_acceleration(self):
        """Message field 'default_acceleration'."""
        return self._default_acceleration

    @default_acceleration.setter
    def default_acceleration(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'default_acceleration' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'default_acceleration' field must be an integer in [-2147483648, 2147483647]"
        self._default_acceleration = value

    @builtins.property
    def max_speed(self):
        """Message field 'max_speed'."""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'max_speed' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'max_speed' field must be an integer in [-2147483648, 2147483647]"
        self._max_speed = value

    @builtins.property
    def buffer_depth(self):
        """Message field 'buffer_depth'."""
        return self._buffer_depth

    @buffer_depth.setter
    def buffer_depth(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'buffer_depth' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'buffer_depth' field must be an integer in [-2147483648, 2147483647]"
        self._buffer_depth = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetMotionParameters_Response(type):
    """Metaclass of message 'SetMotionParameters_Response'."""

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
                'basicmicro_ros2.srv.SetMotionParameters_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_motion_parameters__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_motion_parameters__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_motion_parameters__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_motion_parameters__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_motion_parameters__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetMotionParameters_Response(metaclass=Metaclass_SetMotionParameters_Response):
    """Message class 'SetMotionParameters_Response'."""

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


class Metaclass_SetMotionParameters_Event(type):
    """Metaclass of message 'SetMotionParameters_Event'."""

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
                'basicmicro_ros2.srv.SetMotionParameters_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_motion_parameters__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_motion_parameters__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_motion_parameters__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_motion_parameters__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_motion_parameters__event

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


class SetMotionParameters_Event(metaclass=Metaclass_SetMotionParameters_Event):
    """Message class 'SetMotionParameters_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<basicmicro_ros2/SetMotionParameters_Request, 1>',
        'response': 'sequence<basicmicro_ros2/SetMotionParameters_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['basicmicro_ros2', 'srv'], 'SetMotionParameters_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['basicmicro_ros2', 'srv'], 'SetMotionParameters_Response'), 1),  # noqa: E501
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
            from basicmicro_ros2.srv import SetMotionParameters_Request
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
                 all(isinstance(v, SetMotionParameters_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'SetMotionParameters_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from basicmicro_ros2.srv import SetMotionParameters_Response
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
                 all(isinstance(v, SetMotionParameters_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'SetMotionParameters_Response'"
        self._response = value


class Metaclass_SetMotionParameters(type):
    """Metaclass of service 'SetMotionParameters'."""

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
                'basicmicro_ros2.srv.SetMotionParameters')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_motion_parameters

            from basicmicro_ros2.srv import _set_motion_parameters
            if _set_motion_parameters.Metaclass_SetMotionParameters_Request._TYPE_SUPPORT is None:
                _set_motion_parameters.Metaclass_SetMotionParameters_Request.__import_type_support__()
            if _set_motion_parameters.Metaclass_SetMotionParameters_Response._TYPE_SUPPORT is None:
                _set_motion_parameters.Metaclass_SetMotionParameters_Response.__import_type_support__()
            if _set_motion_parameters.Metaclass_SetMotionParameters_Event._TYPE_SUPPORT is None:
                _set_motion_parameters.Metaclass_SetMotionParameters_Event.__import_type_support__()


class SetMotionParameters(metaclass=Metaclass_SetMotionParameters):
    from basicmicro_ros2.srv._set_motion_parameters import SetMotionParameters_Request as Request
    from basicmicro_ros2.srv._set_motion_parameters import SetMotionParameters_Response as Response
    from basicmicro_ros2.srv._set_motion_parameters import SetMotionParameters_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
