// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inverse_kin:srv/InvKin.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__SRV__DETAIL__INV_KIN__STRUCT_H_
#define INVERSE_KIN__SRV__DETAIL__INV_KIN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/InvKin in the package inverse_kin.
typedef struct inverse_kin__srv__InvKin_Request
{
  double x;
  double y;
  double z;
  double quat_x;
  double quat_y;
  double quat_z;
  double quat_w;
} inverse_kin__srv__InvKin_Request;

// Struct for a sequence of inverse_kin__srv__InvKin_Request.
typedef struct inverse_kin__srv__InvKin_Request__Sequence
{
  inverse_kin__srv__InvKin_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inverse_kin__srv__InvKin_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/InvKin in the package inverse_kin.
typedef struct inverse_kin__srv__InvKin_Response
{
  double q1;
  double q2;
  double q3;
} inverse_kin__srv__InvKin_Response;

// Struct for a sequence of inverse_kin__srv__InvKin_Response.
typedef struct inverse_kin__srv__InvKin_Response__Sequence
{
  inverse_kin__srv__InvKin_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inverse_kin__srv__InvKin_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INVERSE_KIN__SRV__DETAIL__INV_KIN__STRUCT_H_
