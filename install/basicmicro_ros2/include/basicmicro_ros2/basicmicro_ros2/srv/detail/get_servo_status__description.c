// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from basicmicro_ros2:srv/GetServoStatus.idl
// generated code does not contain a copyright notice

#include "basicmicro_ros2/srv/detail/get_servo_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__GetServoStatus__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x77, 0x64, 0xf8, 0x25, 0x24, 0xa1, 0xd4, 0xd9,
      0x83, 0x56, 0x8b, 0xaf, 0x93, 0xd7, 0x27, 0xbe,
      0xdb, 0x3e, 0x0f, 0x6f, 0x2a, 0x55, 0xe0, 0x59,
      0x9a, 0xe2, 0xde, 0x45, 0xce, 0xa9, 0x1e, 0x3f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__GetServoStatus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa1, 0xe3, 0xc4, 0xf1, 0xc0, 0x5f, 0xa0, 0xa2,
      0x55, 0x0b, 0x6f, 0xb3, 0x16, 0x12, 0xe0, 0x82,
      0xb3, 0x27, 0x5f, 0x0c, 0x35, 0x49, 0x22, 0x49,
      0x7e, 0x94, 0x05, 0x45, 0x09, 0xe8, 0x88, 0xf9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__GetServoStatus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xff, 0x5d, 0xe4, 0x07, 0x01, 0xd6, 0x9a, 0x5f,
      0x96, 0xe7, 0x67, 0x33, 0x79, 0x4c, 0x12, 0x14,
      0x42, 0x9a, 0xd0, 0xd9, 0x47, 0x28, 0x3c, 0x68,
      0xcd, 0x84, 0x44, 0xcf, 0x4a, 0xfd, 0x2f, 0x14,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__GetServoStatus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x35, 0xb1, 0x2a, 0x0a, 0xe3, 0x78, 0x9d, 0xc6,
      0xde, 0xef, 0xfa, 0xfc, 0xe9, 0xea, 0x63, 0xa2,
      0x9a, 0x8d, 0x2f, 0x63, 0x95, 0xb1, 0x2d, 0xa0,
      0x63, 0x12, 0xcf, 0x46, 0xf6, 0x51, 0xed, 0x51,
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

static char basicmicro_ros2__srv__GetServoStatus__TYPE_NAME[] = "basicmicro_ros2/srv/GetServoStatus";
static char basicmicro_ros2__srv__GetServoStatus_Event__TYPE_NAME[] = "basicmicro_ros2/srv/GetServoStatus_Event";
static char basicmicro_ros2__srv__GetServoStatus_Request__TYPE_NAME[] = "basicmicro_ros2/srv/GetServoStatus_Request";
static char basicmicro_ros2__srv__GetServoStatus_Response__TYPE_NAME[] = "basicmicro_ros2/srv/GetServoStatus_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char basicmicro_ros2__srv__GetServoStatus__FIELD_NAME__request_message[] = "request_message";
static char basicmicro_ros2__srv__GetServoStatus__FIELD_NAME__response_message[] = "response_message";
static char basicmicro_ros2__srv__GetServoStatus__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__GetServoStatus__FIELDS[] = {
  {
    {basicmicro_ros2__srv__GetServoStatus__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__GetServoStatus_Request__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__GetServoStatus_Response__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__GetServoStatus_Event__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__GetServoStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__GetServoStatus_Event__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Request__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Response__TYPE_NAME, 43, 43},
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
basicmicro_ros2__srv__GetServoStatus__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__GetServoStatus__TYPE_NAME, 34, 34},
      {basicmicro_ros2__srv__GetServoStatus__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__GetServoStatus__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__GetServoStatus_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__GetServoStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = basicmicro_ros2__srv__GetServoStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__GetServoStatus_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__GetServoStatus_Request__FIELDS[] = {
  {
    {basicmicro_ros2__srv__GetServoStatus_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
basicmicro_ros2__srv__GetServoStatus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__GetServoStatus_Request__TYPE_NAME, 42, 42},
      {basicmicro_ros2__srv__GetServoStatus_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__success[] = "success";
static char basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__left_position_error[] = "left_position_error";
static char basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__right_position_error[] = "right_position_error";
static char basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__left_speed_error[] = "left_speed_error";
static char basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__right_speed_error[] = "right_speed_error";
static char basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__error_limits_exceeded[] = "error_limits_exceeded";
static char basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__GetServoStatus_Response__FIELDS[] = {
  {
    {basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__left_position_error, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__right_position_error, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__left_speed_error, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__right_speed_error, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__error_limits_exceeded, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Response__FIELD_NAME__message, 7, 7},
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
basicmicro_ros2__srv__GetServoStatus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__GetServoStatus_Response__TYPE_NAME, 43, 43},
      {basicmicro_ros2__srv__GetServoStatus_Response__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__GetServoStatus_Event__FIELD_NAME__info[] = "info";
static char basicmicro_ros2__srv__GetServoStatus_Event__FIELD_NAME__request[] = "request";
static char basicmicro_ros2__srv__GetServoStatus_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__GetServoStatus_Event__FIELDS[] = {
  {
    {basicmicro_ros2__srv__GetServoStatus_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__GetServoStatus_Request__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__GetServoStatus_Response__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__GetServoStatus_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__GetServoStatus_Request__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetServoStatus_Response__TYPE_NAME, 43, 43},
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
basicmicro_ros2__srv__GetServoStatus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__GetServoStatus_Event__TYPE_NAME, 40, 40},
      {basicmicro_ros2__srv__GetServoStatus_Event__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__GetServoStatus_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__GetServoStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__GetServoStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GetServoStatus.srv\n"
  "# Service to get current servo status and error information\n"
  "\n"
  "# Request\n"
  "# No parameters\n"
  "\n"
  "---\n"
  "\n"
  "# Response\n"
  "bool success                    # True if status was read successfully\n"
  "int32 left_position_error      # Current position error for left motor (encoder counts)\n"
  "int32 right_position_error     # Current position error for right motor (encoder counts)\n"
  "int32 left_speed_error         # Current speed error for left motor (counts/sec)\n"
  "int32 right_speed_error        # Current speed error for right motor (counts/sec)\n"
  "bool error_limits_exceeded     # Whether controller is in error state (requires POR)\n"
  "string message                 # Status message or error description";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__GetServoStatus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__GetServoStatus__TYPE_NAME, 34, 34},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 689, 689},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__GetServoStatus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__GetServoStatus_Request__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__GetServoStatus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__GetServoStatus_Response__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__GetServoStatus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__GetServoStatus_Event__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__GetServoStatus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__GetServoStatus__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__GetServoStatus_Event__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__GetServoStatus_Request__get_individual_type_description_source(NULL);
    sources[3] = *basicmicro_ros2__srv__GetServoStatus_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__GetServoStatus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__GetServoStatus_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__GetServoStatus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__GetServoStatus_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__GetServoStatus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__GetServoStatus_Event__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__GetServoStatus_Request__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__GetServoStatus_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
