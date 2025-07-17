// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inverse_kin:msg/InvKin.idl
// generated code does not contain a copyright notice
#include "inverse_kin/msg/detail/inv_kin__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `scara_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `ee_pos`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `ee_angl`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
inverse_kin__msg__InvKin__init(inverse_kin__msg__InvKin * msg)
{
  if (!msg) {
    return false;
  }
  // scara_pose
  if (!geometry_msgs__msg__Pose__init(&msg->scara_pose)) {
    inverse_kin__msg__InvKin__fini(msg);
    return false;
  }
  // ee_pos
  if (!geometry_msgs__msg__Point__init(&msg->ee_pos)) {
    inverse_kin__msg__InvKin__fini(msg);
    return false;
  }
  // ee_angl
  if (!geometry_msgs__msg__Quaternion__init(&msg->ee_angl)) {
    inverse_kin__msg__InvKin__fini(msg);
    return false;
  }
  return true;
}

void
inverse_kin__msg__InvKin__fini(inverse_kin__msg__InvKin * msg)
{
  if (!msg) {
    return;
  }
  // scara_pose
  geometry_msgs__msg__Pose__fini(&msg->scara_pose);
  // ee_pos
  geometry_msgs__msg__Point__fini(&msg->ee_pos);
  // ee_angl
  geometry_msgs__msg__Quaternion__fini(&msg->ee_angl);
}

bool
inverse_kin__msg__InvKin__are_equal(const inverse_kin__msg__InvKin * lhs, const inverse_kin__msg__InvKin * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // scara_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->scara_pose), &(rhs->scara_pose)))
  {
    return false;
  }
  // ee_pos
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->ee_pos), &(rhs->ee_pos)))
  {
    return false;
  }
  // ee_angl
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->ee_angl), &(rhs->ee_angl)))
  {
    return false;
  }
  return true;
}

bool
inverse_kin__msg__InvKin__copy(
  const inverse_kin__msg__InvKin * input,
  inverse_kin__msg__InvKin * output)
{
  if (!input || !output) {
    return false;
  }
  // scara_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->scara_pose), &(output->scara_pose)))
  {
    return false;
  }
  // ee_pos
  if (!geometry_msgs__msg__Point__copy(
      &(input->ee_pos), &(output->ee_pos)))
  {
    return false;
  }
  // ee_angl
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->ee_angl), &(output->ee_angl)))
  {
    return false;
  }
  return true;
}

inverse_kin__msg__InvKin *
inverse_kin__msg__InvKin__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kin__msg__InvKin * msg = (inverse_kin__msg__InvKin *)allocator.allocate(sizeof(inverse_kin__msg__InvKin), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inverse_kin__msg__InvKin));
  bool success = inverse_kin__msg__InvKin__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inverse_kin__msg__InvKin__destroy(inverse_kin__msg__InvKin * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inverse_kin__msg__InvKin__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inverse_kin__msg__InvKin__Sequence__init(inverse_kin__msg__InvKin__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kin__msg__InvKin * data = NULL;

  if (size) {
    data = (inverse_kin__msg__InvKin *)allocator.zero_allocate(size, sizeof(inverse_kin__msg__InvKin), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inverse_kin__msg__InvKin__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inverse_kin__msg__InvKin__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
inverse_kin__msg__InvKin__Sequence__fini(inverse_kin__msg__InvKin__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      inverse_kin__msg__InvKin__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

inverse_kin__msg__InvKin__Sequence *
inverse_kin__msg__InvKin__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kin__msg__InvKin__Sequence * array = (inverse_kin__msg__InvKin__Sequence *)allocator.allocate(sizeof(inverse_kin__msg__InvKin__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inverse_kin__msg__InvKin__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inverse_kin__msg__InvKin__Sequence__destroy(inverse_kin__msg__InvKin__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inverse_kin__msg__InvKin__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inverse_kin__msg__InvKin__Sequence__are_equal(const inverse_kin__msg__InvKin__Sequence * lhs, const inverse_kin__msg__InvKin__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inverse_kin__msg__InvKin__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inverse_kin__msg__InvKin__Sequence__copy(
  const inverse_kin__msg__InvKin__Sequence * input,
  inverse_kin__msg__InvKin__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inverse_kin__msg__InvKin);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inverse_kin__msg__InvKin * data =
      (inverse_kin__msg__InvKin *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inverse_kin__msg__InvKin__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inverse_kin__msg__InvKin__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inverse_kin__msg__InvKin__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
