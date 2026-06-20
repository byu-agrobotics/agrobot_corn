// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from basicmicro_ros2:srv/GetAvailableHomingMethods.idl
// generated code does not contain a copyright notice

#include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5a, 0xf6, 0x29, 0x79, 0xe6, 0xff, 0x8a, 0xd0,
      0x11, 0xa3, 0x03, 0xac, 0xf7, 0x19, 0x70, 0x7b,
      0x7a, 0xa2, 0xee, 0xa3, 0x55, 0x92, 0xbe, 0xea,
      0xe1, 0xf7, 0x1b, 0x28, 0x36, 0x75, 0x30, 0x99,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xca, 0x92, 0xe4, 0x53, 0x94, 0x2c, 0xdb, 0x2a,
      0x04, 0xe1, 0xa4, 0xe6, 0x79, 0xe2, 0x78, 0x2c,
      0xa3, 0x22, 0x47, 0xc2, 0x76, 0x16, 0x2d, 0xc1,
      0x0a, 0x75, 0x7a, 0xd2, 0x8d, 0x75, 0x99, 0xe2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x45, 0x30, 0xfb, 0x9f, 0x96, 0x65, 0xfc, 0xc3,
      0x66, 0xb5, 0x70, 0xd4, 0xae, 0x98, 0xe1, 0x67,
      0x5d, 0xa7, 0xe7, 0x12, 0xb4, 0xad, 0x13, 0xee,
      0x5f, 0x2e, 0x8d, 0x80, 0xde, 0x28, 0x65, 0xb3,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd6, 0xb1, 0x02, 0x29, 0x01, 0x96, 0xb6, 0xda,
      0x14, 0xab, 0xb8, 0x29, 0x53, 0x0d, 0x6e, 0x7f,
      0xeb, 0xb9, 0x1d, 0xd9, 0xf7, 0xd4, 0x29, 0x85,
      0x81, 0xf3, 0xbd, 0xd9, 0xbd, 0x46, 0xce, 0x33,
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

static char basicmicro_ros2__srv__GetAvailableHomingMethods__TYPE_NAME[] = "basicmicro_ros2/srv/GetAvailableHomingMethods";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Event__TYPE_NAME[] = "basicmicro_ros2/srv/GetAvailableHomingMethods_Event";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Request__TYPE_NAME[] = "basicmicro_ros2/srv/GetAvailableHomingMethods_Request";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Response__TYPE_NAME[] = "basicmicro_ros2/srv/GetAvailableHomingMethods_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char basicmicro_ros2__srv__GetAvailableHomingMethods__FIELD_NAME__request_message[] = "request_message";
static char basicmicro_ros2__srv__GetAvailableHomingMethods__FIELD_NAME__response_message[] = "response_message";
static char basicmicro_ros2__srv__GetAvailableHomingMethods__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__GetAvailableHomingMethods__FIELDS[] = {
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Request__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Event__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__GetAvailableHomingMethods__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Event__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Request__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__TYPE_NAME, 54, 54},
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
basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__GetAvailableHomingMethods__TYPE_NAME, 45, 45},
      {basicmicro_ros2__srv__GetAvailableHomingMethods__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__GetAvailableHomingMethods__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__GetAvailableHomingMethods_Request__FIELDS[] = {
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
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
basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Request__TYPE_NAME, 53, 53},
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__success[] = "success";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__controller_type[] = "controller_type";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__available_methods[] = "available_methods";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__method_descriptions[] = "method_descriptions";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__allowed_directions[] = "allowed_directions";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__auto_zeros_encoder[] = "auto_zeros_encoder";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__acts_as_limit[] = "acts_as_limit";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELDS[] = {
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__controller_type, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__available_methods, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__method_descriptions, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__allowed_directions, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__auto_zeros_encoder, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELD_NAME__acts_as_limit, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__TYPE_NAME, 54, 54},
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Event__FIELD_NAME__info[] = "info";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Event__FIELD_NAME__request[] = "request";
static char basicmicro_ros2__srv__GetAvailableHomingMethods_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__GetAvailableHomingMethods_Event__FIELDS[] = {
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Request__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__GetAvailableHomingMethods_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Request__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__TYPE_NAME, 54, 54},
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
basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Event__TYPE_NAME, 51, 51},
      {basicmicro_ros2__srv__GetAvailableHomingMethods_Event__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GetAvailableHomingMethods.srv\n"
  "# Service to query available homing methods based on controller type and configuration\n"
  "\n"
  "# Request\n"
  "# No parameters\n"
  "\n"
  "---\n"
  "\n"
  "# Response\n"
  "bool success                    # True if query was successful\n"
  "string controller_type         # \"roboclaw\" or \"mcp\"\n"
  "string[] available_methods     # List of available homing method IDs\n"
  "string[] method_descriptions   # Human-readable descriptions for each method\n"
  "string[] allowed_directions    # Allowed directions for each method (forward/backward/both)\n"
  "bool[] auto_zeros_encoder     # Whether each method automatically zeros encoder\n"
  "bool[] acts_as_limit          # Whether each method also acts as limit switch";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__GetAvailableHomingMethods__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__GetAvailableHomingMethods__TYPE_NAME, 45, 45},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 674, 674},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Request__TYPE_NAME, 53, 53},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Response__TYPE_NAME, 54, 54},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__GetAvailableHomingMethods_Event__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__GetAvailableHomingMethods__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_individual_type_description_source(NULL);
    sources[3] = *basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
