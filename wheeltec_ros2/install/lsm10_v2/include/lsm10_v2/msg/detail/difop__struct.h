// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lsm10_v2:msg/Difop.idl
// generated code does not contain a copyright notice

#ifndef LSM10_V2__MSG__DETAIL__DIFOP__STRUCT_H_
#define LSM10_V2__MSG__DETAIL__DIFOP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Difop in the package lsm10_v2.
typedef struct lsm10_v2__msg__Difop
{
  int32_t motorspeed;
} lsm10_v2__msg__Difop;

// Struct for a sequence of lsm10_v2__msg__Difop.
typedef struct lsm10_v2__msg__Difop__Sequence
{
  lsm10_v2__msg__Difop * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lsm10_v2__msg__Difop__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LSM10_V2__MSG__DETAIL__DIFOP__STRUCT_H_
