// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inverse_kin:srv/InvKin.idl
// generated code does not contain a copyright notice
#include "inverse_kin/srv/detail/inv_kin__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
inverse_kin__srv__InvKin_Request__init(inverse_kin__srv__InvKin_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // quat_x
  // quat_y
  // quat_z
  // quat_w
  return true;
}

void
inverse_kin__srv__InvKin_Request__fini(inverse_kin__srv__InvKin_Request * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // quat_x
  // quat_y
  // quat_z
  // quat_w
}

bool
inverse_kin__srv__InvKin_Request__are_equal(const inverse_kin__srv__InvKin_Request * lhs, const inverse_kin__srv__InvKin_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // quat_x
  if (lhs->quat_x != rhs->quat_x) {
    return false;
  }
  // quat_y
  if (lhs->quat_y != rhs->quat_y) {
    return false;
  }
  // quat_z
  if (lhs->quat_z != rhs->quat_z) {
    return false;
  }
  // quat_w
  if (lhs->quat_w != rhs->quat_w) {
    return false;
  }
  return true;
}

bool
inverse_kin__srv__InvKin_Request__copy(
  const inverse_kin__srv__InvKin_Request * input,
  inverse_kin__srv__InvKin_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // quat_x
  output->quat_x = input->quat_x;
  // quat_y
  output->quat_y = input->quat_y;
  // quat_z
  output->quat_z = input->quat_z;
  // quat_w
  output->quat_w = input->quat_w;
  return true;
}

inverse_kin__srv__InvKin_Request *
inverse_kin__srv__InvKin_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kin__srv__InvKin_Request * msg = (inverse_kin__srv__InvKin_Request *)allocator.allocate(sizeof(inverse_kin__srv__InvKin_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inverse_kin__srv__InvKin_Request));
  bool success = inverse_kin__srv__InvKin_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inverse_kin__srv__InvKin_Request__destroy(inverse_kin__srv__InvKin_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inverse_kin__srv__InvKin_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inverse_kin__srv__InvKin_Request__Sequence__init(inverse_kin__srv__InvKin_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kin__srv__InvKin_Request * data = NULL;

  if (size) {
    data = (inverse_kin__srv__InvKin_Request *)allocator.zero_allocate(size, sizeof(inverse_kin__srv__InvKin_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inverse_kin__srv__InvKin_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inverse_kin__srv__InvKin_Request__fini(&data[i - 1]);
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
inverse_kin__srv__InvKin_Request__Sequence__fini(inverse_kin__srv__InvKin_Request__Sequence * array)
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
      inverse_kin__srv__InvKin_Request__fini(&array->data[i]);
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

inverse_kin__srv__InvKin_Request__Sequence *
inverse_kin__srv__InvKin_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kin__srv__InvKin_Request__Sequence * array = (inverse_kin__srv__InvKin_Request__Sequence *)allocator.allocate(sizeof(inverse_kin__srv__InvKin_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inverse_kin__srv__InvKin_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inverse_kin__srv__InvKin_Request__Sequence__destroy(inverse_kin__srv__InvKin_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inverse_kin__srv__InvKin_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inverse_kin__srv__InvKin_Request__Sequence__are_equal(const inverse_kin__srv__InvKin_Request__Sequence * lhs, const inverse_kin__srv__InvKin_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inverse_kin__srv__InvKin_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inverse_kin__srv__InvKin_Request__Sequence__copy(
  const inverse_kin__srv__InvKin_Request__Sequence * input,
  inverse_kin__srv__InvKin_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inverse_kin__srv__InvKin_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inverse_kin__srv__InvKin_Request * data =
      (inverse_kin__srv__InvKin_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inverse_kin__srv__InvKin_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inverse_kin__srv__InvKin_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inverse_kin__srv__InvKin_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
inverse_kin__srv__InvKin_Response__init(inverse_kin__srv__InvKin_Response * msg)
{
  if (!msg) {
    return false;
  }
  // q1
  // q2
  // q3
  return true;
}

void
inverse_kin__srv__InvKin_Response__fini(inverse_kin__srv__InvKin_Response * msg)
{
  if (!msg) {
    return;
  }
  // q1
  // q2
  // q3
}

bool
inverse_kin__srv__InvKin_Response__are_equal(const inverse_kin__srv__InvKin_Response * lhs, const inverse_kin__srv__InvKin_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // q1
  if (lhs->q1 != rhs->q1) {
    return false;
  }
  // q2
  if (lhs->q2 != rhs->q2) {
    return false;
  }
  // q3
  if (lhs->q3 != rhs->q3) {
    return false;
  }
  return true;
}

bool
inverse_kin__srv__InvKin_Response__copy(
  const inverse_kin__srv__InvKin_Response * input,
  inverse_kin__srv__InvKin_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // q1
  output->q1 = input->q1;
  // q2
  output->q2 = input->q2;
  // q3
  output->q3 = input->q3;
  return true;
}

inverse_kin__srv__InvKin_Response *
inverse_kin__srv__InvKin_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kin__srv__InvKin_Response * msg = (inverse_kin__srv__InvKin_Response *)allocator.allocate(sizeof(inverse_kin__srv__InvKin_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inverse_kin__srv__InvKin_Response));
  bool success = inverse_kin__srv__InvKin_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inverse_kin__srv__InvKin_Response__destroy(inverse_kin__srv__InvKin_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inverse_kin__srv__InvKin_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inverse_kin__srv__InvKin_Response__Sequence__init(inverse_kin__srv__InvKin_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kin__srv__InvKin_Response * data = NULL;

  if (size) {
    data = (inverse_kin__srv__InvKin_Response *)allocator.zero_allocate(size, sizeof(inverse_kin__srv__InvKin_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inverse_kin__srv__InvKin_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inverse_kin__srv__InvKin_Response__fini(&data[i - 1]);
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
inverse_kin__srv__InvKin_Response__Sequence__fini(inverse_kin__srv__InvKin_Response__Sequence * array)
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
      inverse_kin__srv__InvKin_Response__fini(&array->data[i]);
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

inverse_kin__srv__InvKin_Response__Sequence *
inverse_kin__srv__InvKin_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kin__srv__InvKin_Response__Sequence * array = (inverse_kin__srv__InvKin_Response__Sequence *)allocator.allocate(sizeof(inverse_kin__srv__InvKin_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inverse_kin__srv__InvKin_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inverse_kin__srv__InvKin_Response__Sequence__destroy(inverse_kin__srv__InvKin_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inverse_kin__srv__InvKin_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inverse_kin__srv__InvKin_Response__Sequence__are_equal(const inverse_kin__srv__InvKin_Response__Sequence * lhs, const inverse_kin__srv__InvKin_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inverse_kin__srv__InvKin_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inverse_kin__srv__InvKin_Response__Sequence__copy(
  const inverse_kin__srv__InvKin_Response__Sequence * input,
  inverse_kin__srv__InvKin_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inverse_kin__srv__InvKin_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inverse_kin__srv__InvKin_Response * data =
      (inverse_kin__srv__InvKin_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inverse_kin__srv__InvKin_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inverse_kin__srv__InvKin_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inverse_kin__srv__InvKin_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
