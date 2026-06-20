// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from basicmicro_ros2:srv/MoveToAbsolutePosition.idl
// generated code does not contain a copyright notice

#include "basicmicro_ros2/srv/detail/move_to_absolute_position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__MoveToAbsolutePosition__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x41, 0xa3, 0x00, 0xa0, 0x06, 0x45, 0x3f, 0x10,
      0xa6, 0xa3, 0x0f, 0x3f, 0x10, 0x1a, 0x89, 0x65,
      0x2a, 0xba, 0x07, 0x90, 0xa1, 0x22, 0xc2, 0xf3,
      0x5b, 0xde, 0x9e, 0x84, 0x43, 0x5b, 0xca, 0x9c,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__MoveToAbsolutePosition_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf7, 0x1e, 0x70, 0x21, 0xdf, 0x08, 0x7d, 0xa4,
      0x1d, 0x8f, 0x52, 0xbe, 0xc3, 0xe5, 0xa4, 0x52,
      0x2c, 0x35, 0x50, 0x9d, 0x2f, 0xe8, 0xc0, 0x97,
      0x19, 0xde, 0x39, 0xf8, 0x59, 0x66, 0x8a, 0x3d,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__MoveToAbsolutePosition_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa4, 0x3a, 0x39, 0x2c, 0x9d, 0x23, 0x00, 0x79,
      0x5a, 0x02, 0xcd, 0x42, 0x0e, 0x4f, 0x47, 0xc6,
      0x7b, 0x6b, 0x9d, 0xe6, 0xb1, 0x36, 0xde, 0x2c,
      0x1a, 0xf6, 0xca, 0x19, 0x2e, 0x98, 0xe6, 0x19,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__MoveToAbsolutePosition_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6a, 0x3d, 0xe1, 0xf4, 0x45, 0xcc, 0xa9, 0xdc,
      0xba, 0x61, 0x1d, 0x85, 0x0c, 0x7c, 0x6c, 0xe3,
      0xa0, 0xfc, 0xdf, 0x6e, 0x75, 0x1e, 0x43, 0xb7,
      0xc5, 0x60, 0x03, 0x62, 0x91, 0x8b, 0x44, 0x21,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char basicmicro_ros2__srv__MoveToAbsolutePosition__TYPE_NAME[] = "basicmicro_ros2/srv/MoveToAbsolutePosition";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Event__TYPE_NAME[] = "basicmicro_ros2/srv/MoveToAbsolutePosition_Event";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Request__TYPE_NAME[] = "basicmicro_ros2/srv/MoveToAbsolutePosition_Request";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Response__TYPE_NAME[] = "basicmicro_ros2/srv/MoveToAbsolutePosition_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char basicmicro_ros2__srv__MoveToAbsolutePosition__FIELD_NAME__request_message[] = "request_message";
static char basicmicro_ros2__srv__MoveToAbsolutePosition__FIELD_NAME__response_message[] = "response_message";
static char basicmicro_ros2__srv__MoveToAbsolutePosition__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__MoveToAbsolutePosition__FIELDS[] = {
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Event__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__MoveToAbsolutePosition__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Event__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Response__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
basicmicro_ros2__srv__MoveToAbsolutePosition__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__MoveToAbsolutePosition__TYPE_NAME, 42, 42},
      {basicmicro_ros2__srv__MoveToAbsolutePosition__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__MoveToAbsolutePosition__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__MoveToAbsolutePosition_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__MoveToAbsolutePosition_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = basicmicro_ros2__srv__MoveToAbsolutePosition_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__left_position_radians[] = "left_position_radians";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__right_position_radians[] = "right_position_radians";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__max_speed[] = "max_speed";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__acceleration[] = "acceleration";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__deceleration[] = "deceleration";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__buffer_command[] = "buffer_command";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELDS[] = {
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__left_position_radians, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__right_position_radians, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__max_speed, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__deceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELD_NAME__buffer_command, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
basicmicro_ros2__srv__MoveToAbsolutePosition_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__TYPE_NAME, 50, 50},
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Response__FIELD_NAME__success[] = "success";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__MoveToAbsolutePosition_Response__FIELDS[] = {
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
basicmicro_ros2__srv__MoveToAbsolutePosition_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Response__TYPE_NAME, 51, 51},
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Event__FIELD_NAME__info[] = "info";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Event__FIELD_NAME__request[] = "request";
static char basicmicro_ros2__srv__MoveToAbsolutePosition_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__MoveToAbsolutePosition_Event__FIELDS[] = {
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__MoveToAbsolutePosition_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Response__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
basicmicro_ros2__srv__MoveToAbsolutePosition_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Event__TYPE_NAME, 48, 48},
      {basicmicro_ros2__srv__MoveToAbsolutePosition_Event__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__MoveToAbsolutePosition_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__MoveToAbsolutePosition_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# MoveToAbsolutePosition.srv\n"
  "# Service for absolute position control (servo mode)\n"
  "\n"
  "# Request\n"
  "float64 left_position_radians    # Absolute position for left motor (radians)\n"
  "float64 right_position_radians   # Absolute position for right motor (radians)\n"
  "float64 max_speed               # Maximum speed during move (m/s)\n"
  "float64 acceleration            # Acceleration rate (m/s\\xc2\\xb2)\n"
  "float64 deceleration            # Deceleration rate (m/s\\xc2\\xb2)\n"
  "bool buffer_command             # Whether to buffer the command for chaining\n"
  "\n"
  "---\n"
  "\n"
  "# Response\n"
  "bool success        # True if command was executed successfully\n"
  "string message      # Status message or error description";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__MoveToAbsolutePosition__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__MoveToAbsolutePosition__TYPE_NAME, 42, 42},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 649, 649},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__MoveToAbsolutePosition_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Request__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__MoveToAbsolutePosition_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Response__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__MoveToAbsolutePosition_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__MoveToAbsolutePosition_Event__TYPE_NAME, 48, 48},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__MoveToAbsolutePosition__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__MoveToAbsolutePosition__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__MoveToAbsolutePosition_Event__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__MoveToAbsolutePosition_Request__get_individual_type_description_source(NULL);
    sources[3] = *basicmicro_ros2__srv__MoveToAbsolutePosition_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__MoveToAbsolutePosition_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__MoveToAbsolutePosition_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__MoveToAbsolutePosition_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__MoveToAbsolutePosition_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__MoveToAbsolutePosition_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__MoveToAbsolutePosition_Event__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__MoveToAbsolutePosition_Request__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__MoveToAbsolutePosition_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
