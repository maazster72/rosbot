// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lsm10_v2:msg/Difop.idl
// generated code does not contain a copyright notice

#ifndef LSM10_V2__MSG__DETAIL__DIFOP__BUILDER_HPP_
#define LSM10_V2__MSG__DETAIL__DIFOP__BUILDER_HPP_

#include "lsm10_v2/msg/detail/difop__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace lsm10_v2
{

namespace msg
{

namespace builder
{

class Init_Difop_motorspeed
{
public:
  Init_Difop_motorspeed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::lsm10_v2::msg::Difop motorspeed(::lsm10_v2::msg::Difop::_motorspeed_type arg)
  {
    msg_.motorspeed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lsm10_v2::msg::Difop msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lsm10_v2::msg::Difop>()
{
  return lsm10_v2::msg::builder::Init_Difop_motorspeed();
}

}  // namespace lsm10_v2

#endif  // LSM10_V2__MSG__DETAIL__DIFOP__BUILDER_HPP_
