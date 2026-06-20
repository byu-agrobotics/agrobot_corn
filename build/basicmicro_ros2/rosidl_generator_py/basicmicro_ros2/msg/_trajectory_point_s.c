// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from basicmicro_ros2:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "basicmicro_ros2/msg/detail/trajectory_point__struct.h"
#include "basicmicro_ros2/msg/detail/trajectory_point__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool basicmicro_ros2__msg__trajectory_point__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
    if (class_attr == NULL) {
      return false;
    }
    PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
    if (name_attr == NULL) {
      Py_DECREF(class_attr);
      return false;
    }
    PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
    if (module_attr == NULL) {
      Py_DECREF(name_attr);
      Py_DECREF(class_attr);
      return false;
    }

    // PyUnicode_1BYTE_DATA is just a cast
    assert(strncmp("basicmicro_ros2.msg._trajectory_point", (char *)PyUnicode_1BYTE_DATA(module_attr), 37) == 0);
    assert(strncmp("TrajectoryPoint", (char *)PyUnicode_1BYTE_DATA(name_attr), 15) == 0);

    Py_DECREF(module_attr);
    Py_DECREF(name_attr);
    Py_DECREF(class_attr);
  }
  basicmicro_ros2__msg__TrajectoryPoint * ros_message = _ros_message;
  {  // command_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "command_type");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->command_type, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // left_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_distance = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_distance = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // deceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "deceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->deceleration = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "acceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->acceleration = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // duration
    PyObject * field = PyObject_GetAttrString(_pymsg, "duration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->duration = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * basicmicro_ros2__msg__trajectory_point__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TrajectoryPoint */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("basicmicro_ros2.msg._trajectory_point");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TrajectoryPoint");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  basicmicro_ros2__msg__TrajectoryPoint * ros_message = (basicmicro_ros2__msg__TrajectoryPoint *)raw_ros_message;
  {  // command_type
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->command_type.data,
      strlen(ros_message->command_type.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "command_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // deceleration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->deceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "deceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acceleration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->acceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // duration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->duration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "duration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
