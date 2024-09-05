// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from lsm10_v2:msg/Difop.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "lsm10_v2/msg/detail/difop__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace lsm10_v2
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Difop_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) lsm10_v2::msg::Difop(_init);
}

void Difop_fini_function(void * message_memory)
{
  auto typed_message = static_cast<lsm10_v2::msg::Difop *>(message_memory);
  typed_message->~Difop();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Difop_message_member_array[1] = {
  {
    "motorspeed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lsm10_v2::msg::Difop, motorspeed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Difop_message_members = {
  "lsm10_v2::msg",  // message namespace
  "Difop",  // message name
  1,  // number of fields
  sizeof(lsm10_v2::msg::Difop),
  Difop_message_member_array,  // message members
  Difop_init_function,  // function to initialize message memory (memory has to be allocated)
  Difop_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Difop_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Difop_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace lsm10_v2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<lsm10_v2::msg::Difop>()
{
  return &::lsm10_v2::msg::rosidl_typesupport_introspection_cpp::Difop_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, lsm10_v2, msg, Difop)() {
  return &::lsm10_v2::msg::rosidl_typesupport_introspection_cpp::Difop_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
