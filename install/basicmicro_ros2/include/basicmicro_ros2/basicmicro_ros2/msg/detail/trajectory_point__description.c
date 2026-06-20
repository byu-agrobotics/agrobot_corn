// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from basicmicro_ros2:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice

#include "basicmicro_ros2/msg/detail/trajectory_point__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__msg__TrajectoryPoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5c, 0x3a, 0x0a, 0xcd, 0x0a, 0x11, 0x34, 0xd9,
      0xd2, 0x6b, 0x30, 0xb6, 0xa0, 0xe8, 0x11, 0x59,
      0x05, 0x30, 0xd5, 0x37, 0x82, 0x5e, 0x63, 0x27,
      0x37, 0x8d, 0x00, 0xab, 0x7b, 0xec, 0xe0, 0x48,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char basicmicro_ros2__msg__TrajectoryPoint__TYPE_NAME[] = "basicmicro_ros2/msg/TrajectoryPoint";

// Define type names, field names, and default values
static char basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__command_type[] = "command_type";
static char basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__left_distance[] = "left_distance";
static char basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__right_distance[] = "right_distance";
static char basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__left_position[] = "left_position";
static char basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__right_position[] = "right_position";
static char basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__deceleration[] = "deceleration";
static char basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__speed[] = "speed";
static char basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__acceleration[] = "acceleration";
static char basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__msg__TrajectoryPoint__FIELDS[] = {
  {
    {basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__command_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__left_distance, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__right_distance, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__left_position, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__right_position, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__deceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__TrajectoryPoint__FIELD_NAME__duration, 8, 8},
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
basicmicro_ros2__msg__TrajectoryPoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__msg__TrajectoryPoint__TYPE_NAME, 35, 35},
      {basicmicro_ros2__msg__TrajectoryPoint__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Trajectory point for mixed distance/position trajectory execution\n"
  "# Supports both relative distance movements and absolute position movements\n"
  "\n"
  "# Command type - determines which fields are used\n"
  "string command_type     # 'distance' or 'position'\n"
  "\n"
  "# For distance commands (relative movement)\n"
  "float64 left_distance    # meters (relative distance to move)\n"
  "float64 right_distance   # meters (relative distance to move)\n"
  "\n"
  "# For position commands (absolute positioning - servo mode)\n"
  "float64 left_position    # radians (absolute wheel position)\n"
  "float64 right_position   # radians (absolute wheel position)\n"
  "float64 deceleration    # m/s\\xc2\\xb2 (deceleration rate for position commands)\n"
  "\n"
  "# Common parameters for both command types\n"
  "float64 speed           # m/s (max speed during movement)\n"
  "float64 acceleration    # m/s\\xc2\\xb2 (acceleration rate)\n"
  "float64 duration        # seconds (optional timing constraint, 0 = no constraint)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__msg__TrajectoryPoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__msg__TrajectoryPoint__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 905, 905},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__msg__TrajectoryPoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__msg__TrajectoryPoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
