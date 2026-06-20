// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from basicmicro_ros2:srv/ExecuteTrajectory.idl
// generated code does not contain a copyright notice

#include "basicmicro_ros2/srv/detail/execute_trajectory__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__ExecuteTrajectory__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4d, 0x0b, 0x53, 0x6a, 0xf5, 0xfa, 0x66, 0x89,
      0x6c, 0x76, 0x23, 0x91, 0x64, 0xb1, 0x93, 0x7f,
      0xca, 0x7f, 0xb3, 0x69, 0x4e, 0x5f, 0x37, 0x50,
      0x7f, 0x0e, 0xa4, 0x9e, 0x72, 0x5f, 0x0c, 0xec,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__ExecuteTrajectory_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2e, 0x20, 0xb6, 0xb7, 0xb0, 0x36, 0x51, 0x02,
      0x22, 0xb3, 0x74, 0x41, 0xef, 0xb5, 0x78, 0x70,
      0xeb, 0x68, 0x7e, 0xcf, 0x2c, 0xa3, 0x33, 0x41,
      0x4b, 0xb9, 0x8d, 0xa3, 0x03, 0xfc, 0xce, 0xd6,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__ExecuteTrajectory_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0a, 0x16, 0x2b, 0x77, 0xf6, 0x37, 0x3a, 0x58,
      0x06, 0x58, 0x86, 0x45, 0xf1, 0x16, 0x84, 0xf3,
      0x10, 0xed, 0x08, 0x0a, 0x46, 0x03, 0x70, 0xbe,
      0x8f, 0xd7, 0x85, 0xe9, 0xfb, 0x74, 0xc0, 0x8b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__srv__ExecuteTrajectory_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x99, 0xa3, 0x8c, 0x52, 0xd7, 0x96, 0xed, 0x16,
      0xa4, 0xaf, 0x47, 0xc9, 0xcd, 0x1b, 0xbe, 0xb4,
      0x08, 0x2e, 0xb8, 0xd0, 0x89, 0x41, 0x86, 0x82,
      0xb2, 0x8c, 0x8f, 0xdd, 0xec, 0xfc, 0x00, 0x47,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "basicmicro_ros2/msg/detail/trajectory_point__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t basicmicro_ros2__msg__TrajectoryPoint__EXPECTED_HASH = {1, {
    0x5c, 0x3a, 0x0a, 0xcd, 0x0a, 0x11, 0x34, 0xd9,
    0xd2, 0x6b, 0x30, 0xb6, 0xa0, 0xe8, 0x11, 0x59,
    0x05, 0x30, 0xd5, 0x37, 0x82, 0x5e, 0x63, 0x27,
    0x37, 0x8d, 0x00, 0xab, 0x7b, 0xec, 0xe0, 0x48,
  }};
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

static char basicmicro_ros2__srv__ExecuteTrajectory__TYPE_NAME[] = "basicmicro_ros2/srv/ExecuteTrajectory";
static char basicmicro_ros2__msg__TrajectoryPoint__TYPE_NAME[] = "basicmicro_ros2/msg/TrajectoryPoint";
static char basicmicro_ros2__srv__ExecuteTrajectory_Event__TYPE_NAME[] = "basicmicro_ros2/srv/ExecuteTrajectory_Event";
static char basicmicro_ros2__srv__ExecuteTrajectory_Request__TYPE_NAME[] = "basicmicro_ros2/srv/ExecuteTrajectory_Request";
static char basicmicro_ros2__srv__ExecuteTrajectory_Response__TYPE_NAME[] = "basicmicro_ros2/srv/ExecuteTrajectory_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char basicmicro_ros2__srv__ExecuteTrajectory__FIELD_NAME__request_message[] = "request_message";
static char basicmicro_ros2__srv__ExecuteTrajectory__FIELD_NAME__response_message[] = "response_message";
static char basicmicro_ros2__srv__ExecuteTrajectory__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__ExecuteTrajectory__FIELDS[] = {
  {
    {basicmicro_ros2__srv__ExecuteTrajectory__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__ExecuteTrajectory_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__ExecuteTrajectory_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {basicmicro_ros2__srv__ExecuteTrajectory_Event__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__ExecuteTrajectory__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__msg__TrajectoryPoint__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Event__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Response__TYPE_NAME, 46, 46},
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
basicmicro_ros2__srv__ExecuteTrajectory__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__ExecuteTrajectory__TYPE_NAME, 37, 37},
      {basicmicro_ros2__srv__ExecuteTrajectory__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__ExecuteTrajectory__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&basicmicro_ros2__msg__TrajectoryPoint__EXPECTED_HASH, basicmicro_ros2__msg__TrajectoryPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__msg__TrajectoryPoint__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__ExecuteTrajectory_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = basicmicro_ros2__srv__ExecuteTrajectory_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = basicmicro_ros2__srv__ExecuteTrajectory_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__ExecuteTrajectory_Request__FIELD_NAME__trajectory_points[] = "trajectory_points";
static char basicmicro_ros2__srv__ExecuteTrajectory_Request__FIELD_NAME__trajectory_type[] = "trajectory_type";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__ExecuteTrajectory_Request__FIELDS[] = {
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Request__FIELD_NAME__trajectory_points, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {basicmicro_ros2__msg__TrajectoryPoint__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Request__FIELD_NAME__trajectory_type, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__ExecuteTrajectory_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__msg__TrajectoryPoint__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
basicmicro_ros2__srv__ExecuteTrajectory_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__ExecuteTrajectory_Request__TYPE_NAME, 45, 45},
      {basicmicro_ros2__srv__ExecuteTrajectory_Request__FIELDS, 2, 2},
    },
    {basicmicro_ros2__srv__ExecuteTrajectory_Request__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&basicmicro_ros2__msg__TrajectoryPoint__EXPECTED_HASH, basicmicro_ros2__msg__TrajectoryPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__msg__TrajectoryPoint__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__ExecuteTrajectory_Response__FIELD_NAME__success[] = "success";
static char basicmicro_ros2__srv__ExecuteTrajectory_Response__FIELD_NAME__message[] = "message";
static char basicmicro_ros2__srv__ExecuteTrajectory_Response__FIELD_NAME__total_commands_sent[] = "total_commands_sent";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__ExecuteTrajectory_Response__FIELDS[] = {
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Response__FIELD_NAME__total_commands_sent, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
basicmicro_ros2__srv__ExecuteTrajectory_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__ExecuteTrajectory_Response__TYPE_NAME, 46, 46},
      {basicmicro_ros2__srv__ExecuteTrajectory_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char basicmicro_ros2__srv__ExecuteTrajectory_Event__FIELD_NAME__info[] = "info";
static char basicmicro_ros2__srv__ExecuteTrajectory_Event__FIELD_NAME__request[] = "request";
static char basicmicro_ros2__srv__ExecuteTrajectory_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__srv__ExecuteTrajectory_Event__FIELDS[] = {
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__ExecuteTrajectory_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {basicmicro_ros2__srv__ExecuteTrajectory_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription basicmicro_ros2__srv__ExecuteTrajectory_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {basicmicro_ros2__msg__TrajectoryPoint__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__srv__ExecuteTrajectory_Response__TYPE_NAME, 46, 46},
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
basicmicro_ros2__srv__ExecuteTrajectory_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__srv__ExecuteTrajectory_Event__TYPE_NAME, 43, 43},
      {basicmicro_ros2__srv__ExecuteTrajectory_Event__FIELDS, 3, 3},
    },
    {basicmicro_ros2__srv__ExecuteTrajectory_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&basicmicro_ros2__msg__TrajectoryPoint__EXPECTED_HASH, basicmicro_ros2__msg__TrajectoryPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = basicmicro_ros2__msg__TrajectoryPoint__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = basicmicro_ros2__srv__ExecuteTrajectory_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = basicmicro_ros2__srv__ExecuteTrajectory_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Execute a trajectory sequence using buffered commands\n"
  "# Supports mixed distance and position trajectory types\n"
  "\n"
  "# Trajectory points to execute in sequence\n"
  "basicmicro_ros2/TrajectoryPoint[] trajectory_points\n"
  "\n"
  "# Trajectory type - controls validation and execution strategy\n"
  "string trajectory_type    # 'distance', 'position', or 'mixed'\n"
  "\n"
  "---\n"
  "# Response\n"
  "bool success                # True if trajectory was successfully queued\n"
  "string message             # Status message or error description  \n"
  "int32 total_commands_sent   # Number of commands successfully sent to buffer";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__ExecuteTrajectory__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__ExecuteTrajectory__TYPE_NAME, 37, 37},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 567, 567},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__ExecuteTrajectory_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__ExecuteTrajectory_Request__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__ExecuteTrajectory_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__ExecuteTrajectory_Response__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__srv__ExecuteTrajectory_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__srv__ExecuteTrajectory_Event__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__ExecuteTrajectory__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__ExecuteTrajectory__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__msg__TrajectoryPoint__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__ExecuteTrajectory_Event__get_individual_type_description_source(NULL);
    sources[3] = *basicmicro_ros2__srv__ExecuteTrajectory_Request__get_individual_type_description_source(NULL);
    sources[4] = *basicmicro_ros2__srv__ExecuteTrajectory_Response__get_individual_type_description_source(NULL);
    sources[5] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__ExecuteTrajectory_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__ExecuteTrajectory_Request__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__msg__TrajectoryPoint__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__ExecuteTrajectory_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__ExecuteTrajectory_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__srv__ExecuteTrajectory_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__srv__ExecuteTrajectory_Event__get_individual_type_description_source(NULL),
    sources[1] = *basicmicro_ros2__msg__TrajectoryPoint__get_individual_type_description_source(NULL);
    sources[2] = *basicmicro_ros2__srv__ExecuteTrajectory_Request__get_individual_type_description_source(NULL);
    sources[3] = *basicmicro_ros2__srv__ExecuteTrajectory_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
