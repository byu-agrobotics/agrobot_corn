// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from basicmicro_ros2:srv/PerformHoming.idl
// generated code does not contain a copyright notice

#include "basicmicro_ros2/srv/detail/perform_homing__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__PerformHoming__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0c, 0x5d, 0x25, 0x12, 0xfb, 0xca, 0xfc, 0xb5,
      0xbf, 0x4e, 0xa2, 0x99, 0xae, 0x09, 0x16, 0xc7,
      0x35, 0x58, 0x77, 0xfa, 0x3b, 0x6e, 0xfd, 0xd8,
      0xdf, 0xb6, 0xc7, 0x85, 0xff, 0x19, 0x22, 0x12,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__PerformHoming_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2b, 0xc8, 0x2c, 0x5e, 0xa8, 0x65, 0x90, 0x61,
      0xaf, 0x85, 0xdf, 0x3b, 0x31, 0x92, 0xe2, 0x7c,
      0x57, 0x67, 0x50, 0x67, 0x7f, 0xa1, 0x93, 0x9b,
      0xbe, 0xcc, 0x6b, 0xc3, 0xe1, 0xf8, 0x59, 0xbe,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__PerformHoming_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe8, 0x98, 0xd6, 0x2f, 0xa7, 0x00, 0x67, 0x27,
      0xa1, 0x8d, 0x02, 0x1a, 0x8c, 0x79, 0x55, 0x74,
      0xa0, 0x62, 0x88, 0x41, 0xe6, 0xf7, 0x13, 0x03,
      0x4c, 0xe6, 0x42, 0x14, 0x17, 0x48, 0xeb, 0xb1,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__PerformHoming_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x14, 0x83, 0x7a, 0x91, 0xbf, 0x06, 0x5f, 0xac,
      0x59, 0x59, 0x30, 0xae, 0xf5, 0xe0, 0x6c, 0x6f,
      0x33, 0x7b, 0x73, 0x2b, 0xc5, 0x3f, 0xae, 0xa0,
      0x6c, 0x8f, 0xe8, 0x55, 0x31, 0x44, 0x9b, 0x85,
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

static char basicmicro_ros2__srv__PerformHoming__TYPE_NAME[] = "basicmicro_ros2/srv/PerformHoming";
static char basicmicro_ros2__srv__PerformHoming_Event__TYPE_NAME[] = "basicmicro_ros2/srv/PerformHoming_Event";
static char basicmicro_ros2__srv__PerformHoming_Request__TYPE_NAME[] = "basicmicro_ros2/srv/PerformHoming_Request";
static char basicmicro_ros2__srv__PerformHoming_Response__TYPE_NAME[] = "basicmicro_ros2/srv/PerformHoming_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char basicmicro_ros2__srv__PerformHoming__FIELD_NAME__request_message[] = "request_message";
static char basicmicro_ros2__srv__PerformHoming__FIELD_NAME__response_message[] = "response_message";
static char basicmicro_ros2__srv__PerformHoming__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__PerformHoming__FIELDS[] = {
  {
    {basicmicro_ros2__srv__PerformHoming__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__PerformHoming_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__PerformHoming_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__PerformHoming_Event__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__PerformHoming__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__PerformHoming_Event__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Response__TYPE_NAME, 42, 42},
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
basicmicro_ros2__srv__PerformHoming__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__PerformHoming__TYPE_NAME, 33, 33},
      {basicmicro_ros2__srv__PerformHoming__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__PerformHoming__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__PerformHoming_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__PerformHoming_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = basicmicro_ros2__srv__PerformHoming_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__PerformHoming_Request__FIELD_NAME__method_id[] = "method_id";
static char basicmicro_ros2__srv__PerformHoming_Request__FIELD_NAME__direction[] = "direction";
static char basicmicro_ros2__srv__PerformHoming_Request__FIELD_NAME__homing_speed[] = "homing_speed";
static char basicmicro_ros2__srv__PerformHoming_Request__FIELD_NAME__timeout[] = "timeout";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__PerformHoming_Request__FIELDS[] = {
  {
    {basicmicro_ros2__srv__PerformHoming_Request__FIELD_NAME__method_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Request__FIELD_NAME__direction, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Request__FIELD_NAME__homing_speed, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Request__FIELD_NAME__timeout, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
basicmicro_ros2__srv__PerformHoming_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__PerformHoming_Request__TYPE_NAME, 41, 41},
      {basicmicro_ros2__srv__PerformHoming_Request__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__PerformHoming_Response__FIELD_NAME__success[] = "success";
static char basicmicro_ros2__srv__PerformHoming_Response__FIELD_NAME__message[] = "message";
static char basicmicro_ros2__srv__PerformHoming_Response__FIELD_NAME__encoder_zeroed[] = "encoder_zeroed";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__PerformHoming_Response__FIELDS[] = {
  {
    {basicmicro_ros2__srv__PerformHoming_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Response__FIELD_NAME__encoder_zeroed, 14, 14},
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
basicmicro_ros2__srv__PerformHoming_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__PerformHoming_Response__TYPE_NAME, 42, 42},
      {basicmicro_ros2__srv__PerformHoming_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__PerformHoming_Event__FIELD_NAME__info[] = "info";
static char basicmicro_ros2__srv__PerformHoming_Event__FIELD_NAME__request[] = "request";
static char basicmicro_ros2__srv__PerformHoming_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__PerformHoming_Event__FIELDS[] = {
  {
    {basicmicro_ros2__srv__PerformHoming_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__PerformHoming_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__PerformHoming_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__PerformHoming_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__PerformHoming_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__PerformHoming_Response__TYPE_NAME, 42, 42},
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
basicmicro_ros2__srv__PerformHoming_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__PerformHoming_Event__TYPE_NAME, 39, 39},
      {basicmicro_ros2__srv__PerformHoming_Event__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__PerformHoming_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__PerformHoming_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__PerformHoming_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# PerformHoming.srv\n"
  "# Service to perform homing sequence using specified method\n"
  "\n"
  "# Request\n"
  "string method_id            # Method ID from available methods\n"
  "string direction           # \"forward\", \"backward\" (if method supports choice)\n"
  "float64 homing_speed       # Homing speed (m/s)\n"
  "float64 timeout            # Timeout for homing operation (seconds)\n"
  "\n"
  "---\n"
  "\n"
  "# Response\n"
  "bool success               # True if homing was successful\n"
  "string message            # Status message or error description\n"
  "bool encoder_zeroed       # Whether encoder was automatically zeroed";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__PerformHoming__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__PerformHoming__TYPE_NAME, 33, 33},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 557, 557},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__PerformHoming_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__PerformHoming_Request__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__PerformHoming_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__PerformHoming_Response__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__PerformHoming_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__PerformHoming_Event__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__PerformHoming__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__PerformHoming__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__PerformHoming_Event__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__PerformHoming_Request__get_individual_type_description_source(NULL);
    sources[3] = *basicmicro_ros2__srv__PerformHoming_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__PerformHoming_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__PerformHoming_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__PerformHoming_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__PerformHoming_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__PerformHoming_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__PerformHoming_Event__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__PerformHoming_Request__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__PerformHoming_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
