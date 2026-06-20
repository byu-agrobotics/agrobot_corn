// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
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
#include "basicmicro_ros2/msg/detail/position_point__struct.h"
#include "basicmicro_ros2/msg/detail/position_point__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool basicmicro_ros2__msg__position_point__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("basicmicro_ros2.msg._position_point", (char *)PyUnicode_1BYTE_DATA(module_attr), 35) == 0);
    assert(strncmp("PositionPoint", (char *)PyUnicode_1BYTE_DATA(name_attr), 13) == 0);

    Py_DECREF(module_attr);
    Py_DECREF(name_attr);
    Py_DECREF(class_attr);
  }
  basicmicro_ros2__msg__PositionPoint * ros_message = _ros_message;
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
  {  // max_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_speed = PyFloat_AS_DOUBLE(field);
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
  {  // deceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "deceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->deceleration = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * basicmicro_ros2__msg__position_point__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PositionPoint */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("basicmicro_ros2.msg._position_point");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PositionPoint");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  basicmicro_ros2__msg__PositionPoint * ros_message = (basicmicro_ros2__msg__PositionPoint *)raw_ros_message;
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
  {  // max_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_speed", field);
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

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
