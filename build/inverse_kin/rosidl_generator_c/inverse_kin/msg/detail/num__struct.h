// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inverse_kin:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__MSG__DETAIL__NUM__STRUCT_H_
#define INVERSE_KIN__MSG__DETAIL__NUM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Num in the package inverse_kin.
typedef struct inverse_kin__msg__Num
{
  int64_t num;
} inverse_kin__msg__Num;

// Struct for a sequence of inverse_kin__msg__Num.
typedef struct inverse_kin__msg__Num__Sequence
{
  inverse_kin__msg__Num * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inverse_kin__msg__Num__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INVERSE_KIN__MSG__DETAIL__NUM__STRUCT_H_
