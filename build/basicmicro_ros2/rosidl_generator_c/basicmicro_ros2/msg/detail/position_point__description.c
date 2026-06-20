// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
// generated code does not contain a copyright notice

#include "basicmicro_ros2/msg/detail/position_point__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_basicmicro_ros2
const rosidl_type_hash_t *
basicmicro_ros2__msg__PositionPoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x59, 0xf4, 0x5f, 0xaa, 0xe0, 0xb2, 0xc3, 0x12,
      0x7d, 0x23, 0xa5, 0x28, 0x68, 0x7a, 0xf3, 0x38,
      0x76, 0xf0, 0xbf, 0xc5, 0x0f, 0x60, 0x4a, 0x0c,
      0xd1, 0xfc, 0xcf, 0xcb, 0xef, 0xf7, 0x69, 0xb1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char basicmicro_ros2__msg__PositionPoint__TYPE_NAME[] = "basicmicro_ros2/msg/PositionPoint";

// Define type names, field names, and default values
static char basicmicro_ros2__msg__PositionPoint__FIELD_NAME__left_position[] = "left_position";
static char basicmicro_ros2__msg__PositionPoint__FIELD_NAME__right_position[] = "right_position";
static char basicmicro_ros2__msg__PositionPoint__FIELD_NAME__max_speed[] = "max_speed";
static char basicmicro_ros2__msg__PositionPoint__FIELD_NAME__acceleration[] = "acceleration";
static char basicmicro_ros2__msg__PositionPoint__FIELD_NAME__deceleration[] = "deceleration";

static rosidl_runtime_c__type_description__Field basicmicro_ros2__msg__PositionPoint__FIELDS[] = {
  {
    {basicmicro_ros2__msg__PositionPoint__FIELD_NAME__left_position, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__PositionPoint__FIELD_NAME__right_position, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__PositionPoint__FIELD_NAME__max_speed, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__PositionPoint__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {basicmicro_ros2__msg__PositionPoint__FIELD_NAME__deceleration, 12, 12},
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
basicmicro_ros2__msg__PositionPoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {basicmicro_ros2__msg__PositionPoint__TYPE_NAME, 33, 33},
      {basicmicro_ros2__msg__PositionPoint__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Position point for servo position sequence execution\n"
  "# Simplified interface for absolute position control\n"
  "\n"
  "float64 left_position     # radians (absolute wheel position)\n"
  "float64 right_position    # radians (absolute wheel position)\n"
  "float64 max_speed        # m/s (maximum speed during movement)\n"
  "float64 acceleration     # m/s\\xc2\\xb2 (acceleration rate)\n"
  "float64 deceleration     # m/s\\xc2\\xb2 (deceleration rate)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
basicmicro_ros2__msg__PositionPoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {basicmicro_ros2__msg__PositionPoint__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 399, 399},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
basicmicro_ros2__msg__PositionPoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *basicmicro_ros2__msg__PositionPoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
