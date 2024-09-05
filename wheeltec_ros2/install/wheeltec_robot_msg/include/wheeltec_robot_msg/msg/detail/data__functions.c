// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wheeltec_robot_msg:msg/Data.idl
// generated code does not contain a copyright notice
#include "wheeltec_robot_msg/msg/detail/data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
wheeltec_robot_msg__msg__Data__init(wheeltec_robot_msg__msg__Data * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  return true;
}

void
wheeltec_robot_msg__msg__Data__fini(wheeltec_robot_msg__msg__Data * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
}

bool
wheeltec_robot_msg__msg__Data__are_equal(const wheeltec_robot_msg__msg__Data * lhs, const wheeltec_robot_msg__msg__Data * rhs)
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
  return true;
}

bool
wheeltec_robot_msg__msg__Data__copy(
  const wheeltec_robot_msg__msg__Data * input,
  wheeltec_robot_msg__msg__Data * output)
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
  return true;
}

wheeltec_robot_msg__msg__Data *
wheeltec_robot_msg__msg__Data__create()
{
  wheeltec_robot_msg__msg__Data * msg = (wheeltec_robot_msg__msg__Data *)malloc(sizeof(wheeltec_robot_msg__msg__Data));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheeltec_robot_msg__msg__Data));
  bool success = wheeltec_robot_msg__msg__Data__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
wheeltec_robot_msg__msg__Data__destroy(wheeltec_robot_msg__msg__Data * msg)
{
  if (msg) {
    wheeltec_robot_msg__msg__Data__fini(msg);
  }
  free(msg);
}


bool
wheeltec_robot_msg__msg__Data__Sequence__init(wheeltec_robot_msg__msg__Data__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  wheeltec_robot_msg__msg__Data * data = NULL;
  if (size) {
    data = (wheeltec_robot_msg__msg__Data *)calloc(size, sizeof(wheeltec_robot_msg__msg__Data));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheeltec_robot_msg__msg__Data__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheeltec_robot_msg__msg__Data__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
wheeltec_robot_msg__msg__Data__Sequence__fini(wheeltec_robot_msg__msg__Data__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      wheeltec_robot_msg__msg__Data__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

wheeltec_robot_msg__msg__Data__Sequence *
wheeltec_robot_msg__msg__Data__Sequence__create(size_t size)
{
  wheeltec_robot_msg__msg__Data__Sequence * array = (wheeltec_robot_msg__msg__Data__Sequence *)malloc(sizeof(wheeltec_robot_msg__msg__Data__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = wheeltec_robot_msg__msg__Data__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
wheeltec_robot_msg__msg__Data__Sequence__destroy(wheeltec_robot_msg__msg__Data__Sequence * array)
{
  if (array) {
    wheeltec_robot_msg__msg__Data__Sequence__fini(array);
  }
  free(array);
}

bool
wheeltec_robot_msg__msg__Data__Sequence__are_equal(const wheeltec_robot_msg__msg__Data__Sequence * lhs, const wheeltec_robot_msg__msg__Data__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheeltec_robot_msg__msg__Data__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheeltec_robot_msg__msg__Data__Sequence__copy(
  const wheeltec_robot_msg__msg__Data__Sequence * input,
  wheeltec_robot_msg__msg__Data__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheeltec_robot_msg__msg__Data);
    wheeltec_robot_msg__msg__Data * data =
      (wheeltec_robot_msg__msg__Data *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheeltec_robot_msg__msg__Data__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          wheeltec_robot_msg__msg__Data__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheeltec_robot_msg__msg__Data__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
