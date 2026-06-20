# generated from rosidl_generator_py/resource/_idl.py.em
# with input from basicmicro_ros2:srv/GetAvailableHomingMethods.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetAvailableHomingMethods_Request(type):
    """Metaclass of message 'GetAvailableHomingMethods_Request'."""

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
                'basicmicro_ros2.srv.GetAvailableHomingMethods_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_available_homing_methods__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_available_homing_methods__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_available_homing_methods__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_available_homing_methods__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_available_homing_methods__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetAvailableHomingMethods_Request(metaclass=Metaclass_GetAvailableHomingMethods_Request):
    """Message class 'GetAvailableHomingMethods_Request'."""

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


class Metaclass_GetAvailableHomingMethods_Response(type):
    """Metaclass of message 'GetAvailableHomingMethods_Response'."""

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
                'basicmicro_ros2.srv.GetAvailableHomingMethods_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_available_homing_methods__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_available_homing_methods__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_available_homing_methods__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_available_homing_methods__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_available_homing_methods__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetAvailableHomingMethods_Response(metaclass=Metaclass_GetAvailableHomingMethods_Response):
    """Message class 'GetAvailableHomingMethods_Response'."""

    __slots__ = [
        '_success',
        '_controller_type',
        '_available_methods',
        '_method_descriptions',
        '_allowed_directions',
        '_auto_zeros_encoder',
        '_acts_as_limit',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'controller_type': 'string',
        'available_methods': 'sequence<string>',
        'method_descriptions': 'sequence<string>',
        'allowed_directions': 'sequence<string>',
        'auto_zeros_encoder': 'sequence<boolean>',
        'acts_as_limit': 'sequence<boolean>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
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
        self.controller_type = kwargs.get('controller_type', str())
        self.available_methods = kwargs.get('available_methods', [])
        self.method_descriptions = kwargs.get('method_descriptions', [])
        self.allowed_directions = kwargs.get('allowed_directions', [])
        self.auto_zeros_encoder = kwargs.get('auto_zeros_encoder', [])
        self.acts_as_limit = kwargs.get('acts_as_limit', [])

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
        if self.controller_type != other.controller_type:
            return False
        if self.available_methods != other.available_methods:
            return False
        if self.method_descriptions != other.method_descriptions:
            return False
        if self.allowed_directions != other.allowed_directions:
            return False
        if self.auto_zeros_encoder != other.auto_zeros_encoder:
            return False
        if self.acts_as_limit != other.acts_as_limit:
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
    def controller_type(self):
        """Message field 'controller_type'."""
        return self._controller_type

    @controller_type.setter
    def controller_type(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'controller_type' field must be of type 'str'"
        self._controller_type = value

    @builtins.property
    def available_methods(self):
        """Message field 'available_methods'."""
        return self._available_methods

    @available_methods.setter
    def available_methods(self, value):
        if self._check_fields:
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'available_methods' field must be a set or sequence and each value of type 'str'"
        self._available_methods = value

    @builtins.property
    def method_descriptions(self):
        """Message field 'method_descriptions'."""
        return self._method_descriptions

    @method_descriptions.setter
    def method_descriptions(self, value):
        if self._check_fields:
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'method_descriptions' field must be a set or sequence and each value of type 'str'"
        self._method_descriptions = value

    @builtins.property
    def allowed_directions(self):
        """Message field 'allowed_directions'."""
        return self._allowed_directions

    @allowed_directions.setter
    def allowed_directions(self, value):
        if self._check_fields:
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'allowed_directions' field must be a set or sequence and each value of type 'str'"
        self._allowed_directions = value

    @builtins.property
    def auto_zeros_encoder(self):
        """Message field 'auto_zeros_encoder'."""
        return self._auto_zeros_encoder

    @auto_zeros_encoder.setter
    def auto_zeros_encoder(self, value):
        if self._check_fields:
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'auto_zeros_encoder' field must be a set or sequence and each value of type 'bool'"
        self._auto_zeros_encoder = value

    @builtins.property
    def acts_as_limit(self):
        """Message field 'acts_as_limit'."""
        return self._acts_as_limit

    @acts_as_limit.setter
    def acts_as_limit(self, value):
        if self._check_fields:
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'acts_as_limit' field must be a set or sequence and each value of type 'bool'"
        self._acts_as_limit = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetAvailableHomingMethods_Event(type):
    """Metaclass of message 'GetAvailableHomingMethods_Event'."""

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
                'basicmicro_ros2.srv.GetAvailableHomingMethods_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_available_homing_methods__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_available_homing_methods__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_available_homing_methods__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_available_homing_methods__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_available_homing_methods__event

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


class GetAvailableHomingMethods_Event(metaclass=Metaclass_GetAvailableHomingMethods_Event):
    """Message class 'GetAvailableHomingMethods_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<basicmicro_ros2/GetAvailableHomingMethods_Request, 1>',
        'response': 'sequence<basicmicro_ros2/GetAvailableHomingMethods_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['basicmicro_ros2', 'srv'], 'GetAvailableHomingMethods_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['basicmicro_ros2', 'srv'], 'GetAvailableHomingMethods_Response'), 1),  # noqa: E501
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
            from basicmicro_ros2.srv import GetAvailableHomingMethods_Request
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
                 all(isinstance(v, GetAvailableHomingMethods_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'GetAvailableHomingMethods_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from basicmicro_ros2.srv import GetAvailableHomingMethods_Response
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
                 all(isinstance(v, GetAvailableHomingMethods_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'GetAvailableHomingMethods_Response'"
        self._response = value


class Metaclass_GetAvailableHomingMethods(type):
    """Metaclass of service 'GetAvailableHomingMethods'."""

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
                'basicmicro_ros2.srv.GetAvailableHomingMethods')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_available_homing_methods

            from basicmicro_ros2.srv import _get_available_homing_methods
            if _get_available_homing_methods.Metaclass_GetAvailableHomingMethods_Request._TYPE_SUPPORT is None:
                _get_available_homing_methods.Metaclass_GetAvailableHomingMethods_Request.__import_type_support__()
            if _get_available_homing_methods.Metaclass_GetAvailableHomingMethods_Response._TYPE_SUPPORT is None:
                _get_available_homing_methods.Metaclass_GetAvailableHomingMethods_Response.__import_type_support__()
            if _get_available_homing_methods.Metaclass_GetAvailableHomingMethods_Event._TYPE_SUPPORT is None:
                _get_available_homing_methods.Metaclass_GetAvailableHomingMethods_Event.__import_type_support__()


class GetAvailableHomingMethods(metaclass=Metaclass_GetAvailableHomingMethods):
    from basicmicro_ros2.srv._get_available_homing_methods import GetAvailableHomingMethods_Request as Request
    from basicmicro_ros2.srv._get_available_homing_methods import GetAvailableHomingMethods_Response as Response
    from basicmicro_ros2.srv._get_available_homing_methods import GetAvailableHomingMethods_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
