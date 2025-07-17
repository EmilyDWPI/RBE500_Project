// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inverse_kin:msg/InvKin.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__MSG__DETAIL__INV_KIN__STRUCT_H_
#define INVERSE_KIN__MSG__DETAIL__INV_KIN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'scara_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'ee_pos'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'ee_angl'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/InvKin in the package inverse_kin.
typedef struct inverse_kin__msg__InvKin
{
  geometry_msgs__msg__Pose scara_pose;
  geometry_msgs__msg__Point ee_pos;
  geometry_msgs__msg__Quaternion ee_angl;
} inverse_kin__msg__InvKin;

// Struct for a sequence of inverse_kin__msg__InvKin.
typedef struct inverse_kin__msg__InvKin__Sequence
{
  inverse_kin__msg__InvKin * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inverse_kin__msg__InvKin__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INVERSE_KIN__MSG__DETAIL__INV_KIN__STRUCT_H_
