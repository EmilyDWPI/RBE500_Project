// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from inverse_kin:srv/InvKin.idl
// generated code does not contain a copyright notice
#include "inverse_kin/srv/detail/inv_kin__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "inverse_kin/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "inverse_kin/srv/detail/inv_kin__struct.h"
#include "inverse_kin/srv/detail/inv_kin__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _InvKin_Request__ros_msg_type = inverse_kin__srv__InvKin_Request;

static bool _InvKin_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _InvKin_Request__ros_msg_type * ros_message = static_cast<const _InvKin_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: x
  {
    cdr << ros_message->x;
  }

  // Field name: y
  {
    cdr << ros_message->y;
  }

  // Field name: z
  {
    cdr << ros_message->z;
  }

  // Field name: quat_x
  {
    cdr << ros_message->quat_x;
  }

  // Field name: quat_y
  {
    cdr << ros_message->quat_y;
  }

  // Field name: quat_z
  {
    cdr << ros_message->quat_z;
  }

  // Field name: quat_w
  {
    cdr << ros_message->quat_w;
  }

  return true;
}

static bool _InvKin_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _InvKin_Request__ros_msg_type * ros_message = static_cast<_InvKin_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: x
  {
    cdr >> ros_message->x;
  }

  // Field name: y
  {
    cdr >> ros_message->y;
  }

  // Field name: z
  {
    cdr >> ros_message->z;
  }

  // Field name: quat_x
  {
    cdr >> ros_message->quat_x;
  }

  // Field name: quat_y
  {
    cdr >> ros_message->quat_y;
  }

  // Field name: quat_z
  {
    cdr >> ros_message->quat_z;
  }

  // Field name: quat_w
  {
    cdr >> ros_message->quat_w;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inverse_kin
size_t get_serialized_size_inverse_kin__srv__InvKin_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _InvKin_Request__ros_msg_type * ros_message = static_cast<const _InvKin_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name x
  {
    size_t item_size = sizeof(ros_message->x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y
  {
    size_t item_size = sizeof(ros_message->y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name z
  {
    size_t item_size = sizeof(ros_message->z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name quat_x
  {
    size_t item_size = sizeof(ros_message->quat_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name quat_y
  {
    size_t item_size = sizeof(ros_message->quat_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name quat_z
  {
    size_t item_size = sizeof(ros_message->quat_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name quat_w
  {
    size_t item_size = sizeof(ros_message->quat_w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _InvKin_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_inverse_kin__srv__InvKin_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inverse_kin
size_t max_serialized_size_inverse_kin__srv__InvKin_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: quat_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: quat_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: quat_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: quat_w
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = inverse_kin__srv__InvKin_Request;
    is_plain =
      (
      offsetof(DataType, quat_w) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _InvKin_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_inverse_kin__srv__InvKin_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_InvKin_Request = {
  "inverse_kin::srv",
  "InvKin_Request",
  _InvKin_Request__cdr_serialize,
  _InvKin_Request__cdr_deserialize,
  _InvKin_Request__get_serialized_size,
  _InvKin_Request__max_serialized_size
};

static rosidl_message_type_support_t _InvKin_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_InvKin_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inverse_kin, srv, InvKin_Request)() {
  return &_InvKin_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "inverse_kin/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "inverse_kin/srv/detail/inv_kin__struct.h"
// already included above
// #include "inverse_kin/srv/detail/inv_kin__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _InvKin_Response__ros_msg_type = inverse_kin__srv__InvKin_Response;

static bool _InvKin_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _InvKin_Response__ros_msg_type * ros_message = static_cast<const _InvKin_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: q1
  {
    cdr << ros_message->q1;
  }

  // Field name: q2
  {
    cdr << ros_message->q2;
  }

  // Field name: q3
  {
    cdr << ros_message->q3;
  }

  return true;
}

static bool _InvKin_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _InvKin_Response__ros_msg_type * ros_message = static_cast<_InvKin_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: q1
  {
    cdr >> ros_message->q1;
  }

  // Field name: q2
  {
    cdr >> ros_message->q2;
  }

  // Field name: q3
  {
    cdr >> ros_message->q3;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inverse_kin
size_t get_serialized_size_inverse_kin__srv__InvKin_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _InvKin_Response__ros_msg_type * ros_message = static_cast<const _InvKin_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name q1
  {
    size_t item_size = sizeof(ros_message->q1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name q2
  {
    size_t item_size = sizeof(ros_message->q2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name q3
  {
    size_t item_size = sizeof(ros_message->q3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _InvKin_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_inverse_kin__srv__InvKin_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inverse_kin
size_t max_serialized_size_inverse_kin__srv__InvKin_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: q1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: q2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: q3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = inverse_kin__srv__InvKin_Response;
    is_plain =
      (
      offsetof(DataType, q3) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _InvKin_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_inverse_kin__srv__InvKin_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_InvKin_Response = {
  "inverse_kin::srv",
  "InvKin_Response",
  _InvKin_Response__cdr_serialize,
  _InvKin_Response__cdr_deserialize,
  _InvKin_Response__get_serialized_size,
  _InvKin_Response__max_serialized_size
};

static rosidl_message_type_support_t _InvKin_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_InvKin_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inverse_kin, srv, InvKin_Response)() {
  return &_InvKin_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "inverse_kin/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "inverse_kin/srv/inv_kin.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t InvKin__callbacks = {
  "inverse_kin::srv",
  "InvKin",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inverse_kin, srv, InvKin_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inverse_kin, srv, InvKin_Response)(),
};

static rosidl_service_type_support_t InvKin__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &InvKin__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inverse_kin, srv, InvKin)() {
  return &InvKin__handle;
}

#if defined(__cplusplus)
}
#endif
