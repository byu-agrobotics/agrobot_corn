// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from basicmicro_ros2:srv/SetMotionStrategy.idl
// generated code does not contain a copyright notice

#include "basicmicro_ros2/srv/detail/set_motion_strategy__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__SetMotionStrategy__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd5, 0x9a, 0x5f, 0xde, 0x44, 0x05, 0x56, 0xc8,
      0x40, 0x94, 0xcf, 0x92, 0xaa, 0x7b, 0x73, 0x04,
      0xb3, 0x57, 0x1c, 0x52, 0xd3, 0x8e, 0x60, 0xb2,
      0x2a, 0xa0, 0xdb, 0x7d, 0x1d, 0x97, 0xa6, 0x10,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__SetMotionStrategy_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x74, 0x03, 0x7e, 0x58, 0xfc, 0x96, 0x88, 0xd0,
      0x1c, 0x8e, 0xaf, 0x60, 0xee, 0x4a, 0x13, 0xf3,
      0x59, 0xa6, 0xe9, 0xbe, 0xa3, 0x1b, 0xe6, 0x15,
      0x40, 0x01, 0x28, 0x92, 0x8e, 0x54, 0x93, 0x31,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__SetMotionStrategy_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4e, 0x80, 0xb9, 0xb4, 0x46, 0x71, 0x8f, 0x00,
      0x53, 0x82, 0xe7, 0x78, 0x5d, 0x84, 0x9a, 0x8f,
      0xe5, 0x4d, 0x5a, 0xb5, 0x72, 0x4e, 0x2e, 0x21,
      0xee, 0x1d, 0x15, 0xe9, 0xdf, 0x6c, 0x8e, 0x0d,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__SetMotionStrategy_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa9, 0x04, 0x2f, 0x8d, 0x79, 0x88, 0xd1, 0x43,
      0xcf, 0x35, 0x4c, 0xf8, 0x6f, 0xd2, 0x49, 0x35,
      0xa9, 0x80, 0x24, 0xce, 0xa2, 0xcc, 0xe6, 0x55,
      0x6e, 0x57, 0x4a, 0xf8, 0x50, 0x72, 0x27, 0x1e,
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

static char basicmicro_ros2__srv__SetMotionStrategy__TYPE_NAME[] = "basicmicro_ros2/srv/SetMotionStrategy";
static char basicmicro_ros2__srv__SetMotionStrategy_Event__TYPE_NAME[] = "basicmicro_ros2/srv/SetMotionStrategy_Event";
static char basicmicro_ros2__srv__SetMotionStrategy_Request__TYPE_NAME[] = "basicmicro_ros2/srv/SetMotionStrategy_Request";
static char basicmicro_ros2__srv__SetMotionStrategy_Response__TYPE_NAME[] = "basicmicro_ros2/srv/SetMotionStrategy_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char basicmicro_ros2__srv__SetMotionStrategy__FIELD_NAME__request_message[] = "request_message";
static char basicmicro_ros2__srv__SetMotionStrategy__FIELD_NAME__response_message[] = "response_message";
static char basicmicro_ros2__srv__SetMotionStrategy__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__SetMotionStrategy__FIELDS[] = {
  {
    {basicmicro_ros2__srv__SetMotionStrategy__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__SetMotionStrategy_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__SetMotionStrategy__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__SetMotionStrategy_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__SetMotionStrategy__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__SetMotionStrategy_Event__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__SetMotionStrategy__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Event__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Response__TYPE_NAME, 46, 46},
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
basicmicro_ros2__srv__SetMotionStrategy__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__SetMotionStrategy__TYPE_NAME, 37, 37},
      {basicmicro_ros2__srv__SetMotionStrategy__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__SetMotionStrategy__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__SetMotionStrategy_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__SetMotionStrategy_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = basicmicro_ros2__srv__SetMotionStrategy_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__SetMotionStrategy_Request__FIELD_NAME__strategy[] = "strategy";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__SetMotionStrategy_Request__FIELDS[] = {
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Request__FIELD_NAME__strategy, 8, 8},
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
basicmicro_ros2__srv__SetMotionStrategy_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__SetMotionStrategy_Request__TYPE_NAME, 45, 45},
      {basicmicro_ros2__srv__SetMotionStrategy_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__SetMotionStrategy_Response__FIELD_NAME__success[] = "success";
static char basicmicro_ros2__srv__SetMotionStrategy_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__SetMotionStrategy_Response__FIELDS[] = {
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Response__FIELD_NAME__message, 7, 7},
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
basicmicro_ros2__srv__SetMotionStrategy_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__SetMotionStrategy_Response__TYPE_NAME, 46, 46},
      {basicmicro_ros2__srv__SetMotionStrategy_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__SetMotionStrategy_Event__FIELD_NAME__info[] = "info";
static char basicmicro_ros2__srv__SetMotionStrategy_Event__FIELD_NAME__request[] = "request";
static char basicmicro_ros2__srv__SetMotionStrategy_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__SetMotionStrategy_Event__FIELDS[] = {
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__SetMotionStrategy_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__SetMotionStrategy_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__SetMotionStrategy_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__SetMotionStrategy_Response__TYPE_NAME, 46, 46},
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
basicmicro_ros2__srv__SetMotionStrategy_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__SetMotionStrategy_Event__TYPE_NAME, 43, 43},
      {basicmicro_ros2__srv__SetMotionStrategy_Event__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__SetMotionStrategy_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__SetMotionStrategy_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__SetMotionStrategy_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Service to set the motion strategy at runtime\n"
  "# Supported strategies: 'duty', 'duty_accel', 'speed', 'speed_accel'\n"
  "\n"
  "string strategy  # Motion strategy name\n"
  "---\n"
  "bool success     # Whether the strategy was set successfully\n"
  "string message   # Status message or error description";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__SetMotionStrategy__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__SetMotionStrategy__TYPE_NAME, 37, 37},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 277, 277},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__SetMotionStrategy_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__SetMotionStrategy_Request__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__SetMotionStrategy_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__SetMotionStrategy_Response__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__SetMotionStrategy_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__SetMotionStrategy_Event__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__SetMotionStrategy__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__SetMotionStrategy__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__SetMotionStrategy_Event__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__SetMotionStrategy_Request__get_individual_type_description_source(NULL);
    sources[3] = *basicmicro_ros2__srv__SetMotionStrategy_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__SetMotionStrategy_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__SetMotionStrategy_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__SetMotionStrategy_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__SetMotionStrategy_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__SetMotionStrategy_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__SetMotionStrategy_Event__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__SetMotionStrategy_Request__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__SetMotionStrategy_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
