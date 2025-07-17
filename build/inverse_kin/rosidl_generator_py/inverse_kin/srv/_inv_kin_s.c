// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from inverse_kin:srv/InvKin.idl
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
#include "inverse_kin/srv/detail/inv_kin__struct.h"
#include "inverse_kin/srv/detail/inv_kin__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool inverse_kin__srv__inv_kin__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("inverse_kin.srv._inv_kin.InvKin_Request", full_classname_dest, 39) == 0);
  }
  inverse_kin__srv__InvKin_Request * ros_message = _ros_message;
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quat_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "quat_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quat_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quat_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "quat_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quat_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quat_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "quat_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quat_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quat_w
    PyObject * field = PyObject_GetAttrString(_pymsg, "quat_w");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quat_w = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inverse_kin__srv__inv_kin__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of InvKin_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inverse_kin.srv._inv_kin");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "InvKin_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inverse_kin__srv__InvKin_Request * ros_message = (inverse_kin__srv__InvKin_Request *)raw_ros_message;
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quat_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quat_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quat_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quat_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quat_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quat_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quat_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quat_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quat_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quat_w
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quat_w);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quat_w", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "inverse_kin/srv/detail/inv_kin__struct.h"
// already included above
// #include "inverse_kin/srv/detail/inv_kin__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool inverse_kin__srv__inv_kin__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
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
    assert(strncmp("inverse_kin.srv._inv_kin.InvKin_Response", full_classname_dest, 40) == 0);
  }
  inverse_kin__srv__InvKin_Response * ros_message = _ros_message;
  {  // q1
    PyObject * field = PyObject_GetAttrString(_pymsg, "q1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->q1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // q2
    PyObject * field = PyObject_GetAttrString(_pymsg, "q2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->q2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // q3
    PyObject * field = PyObject_GetAttrString(_pymsg, "q3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->q3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * inverse_kin__srv__inv_kin__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of InvKin_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("inverse_kin.srv._inv_kin");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "InvKin_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  inverse_kin__srv__InvKin_Response * ros_message = (inverse_kin__srv__InvKin_Response *)raw_ros_message;
  {  // q1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->q1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "q1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // q2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->q2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "q2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // q3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->q3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "q3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
