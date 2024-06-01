// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from pplanner_interfaces:msg/ArucoData.idl
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
#include "pplanner_interfaces/msg/detail/aruco_data__struct.h"
#include "pplanner_interfaces/msg/detail/aruco_data__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool pplanner_interfaces__msg__aruco_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("pplanner_interfaces.msg._aruco_data.ArucoData", full_classname_dest, 45) == 0);
  }
  pplanner_interfaces__msg__ArucoData * ros_message = _ros_message;
  {  // id_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "id_data");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id_data = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // x_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_data");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->x_data = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // y_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_data");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->y_data = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // orientation_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "orientation_data");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->orientation_data = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->status, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * pplanner_interfaces__msg__aruco_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ArucoData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("pplanner_interfaces.msg._aruco_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ArucoData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  pplanner_interfaces__msg__ArucoData * ros_message = (pplanner_interfaces__msg__ArucoData *)raw_ros_message;
  {  // id_data
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->id_data);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_data
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->x_data);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_data
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->y_data);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // orientation_data
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->orientation_data);
    {
      int rc = PyObject_SetAttrString(_pymessage, "orientation_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // status
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->status.data,
      strlen(ros_message->status.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
