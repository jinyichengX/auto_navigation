// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from status_interface:msg/SystemStatus.idl
// generated code does not contain a copyright notice

#ifndef STATUS_INTERFACE__MSG__DETAIL__SYSTEM_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define STATUS_INTERFACE__MSG__DETAIL__SYSTEM_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "status_interface/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "status_interface/msg/detail/system_status__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace status_interface
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_status_interface
cdr_serialize(
  const status_interface::msg::SystemStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_status_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  status_interface::msg::SystemStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_status_interface
get_serialized_size(
  const status_interface::msg::SystemStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_status_interface
max_serialized_size_SystemStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace status_interface

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_status_interface
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, status_interface, msg, SystemStatus)();

#ifdef __cplusplus
}
#endif

#endif  // STATUS_INTERFACE__MSG__DETAIL__SYSTEM_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
