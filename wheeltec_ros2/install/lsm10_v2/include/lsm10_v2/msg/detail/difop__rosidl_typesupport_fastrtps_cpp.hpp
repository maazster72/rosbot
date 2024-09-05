// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from lsm10_v2:msg/Difop.idl
// generated code does not contain a copyright notice

#ifndef LSM10_V2__MSG__DETAIL__DIFOP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define LSM10_V2__MSG__DETAIL__DIFOP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "lsm10_v2/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "lsm10_v2/msg/detail/difop__struct.hpp"

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

namespace lsm10_v2
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lsm10_v2
cdr_serialize(
  const lsm10_v2::msg::Difop & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lsm10_v2
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  lsm10_v2::msg::Difop & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lsm10_v2
get_serialized_size(
  const lsm10_v2::msg::Difop & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lsm10_v2
max_serialized_size_Difop(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace lsm10_v2

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lsm10_v2
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, lsm10_v2, msg, Difop)();

#ifdef __cplusplus
}
#endif

#endif  // LSM10_V2__MSG__DETAIL__DIFOP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
