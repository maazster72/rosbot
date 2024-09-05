// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astra_camera_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice
#include "astra_camera_msgs/msg/detail/metadata__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `json_data`
#include "rosidl_runtime_c/string_functions.h"

bool
astra_camera_msgs__msg__Metadata__init(astra_camera_msgs__msg__Metadata * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    astra_camera_msgs__msg__Metadata__fini(msg);
    return false;
  }
  // json_data
  if (!rosidl_runtime_c__String__init(&msg->json_data)) {
    astra_camera_msgs__msg__Metadata__fini(msg);
    return false;
  }
  return true;
}

void
astra_camera_msgs__msg__Metadata__fini(astra_camera_msgs__msg__Metadata * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // json_data
  rosidl_runtime_c__String__fini(&msg->json_data);
}

bool
astra_camera_msgs__msg__Metadata__are_equal(const astra_camera_msgs__msg__Metadata * lhs, const astra_camera_msgs__msg__Metadata * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // json_data
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->json_data), &(rhs->json_data)))
  {
    return false;
  }
  return true;
}

bool
astra_camera_msgs__msg__Metadata__copy(
  const astra_camera_msgs__msg__Metadata * input,
  astra_camera_msgs__msg__Metadata * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // json_data
  if (!rosidl_runtime_c__String__copy(
      &(input->json_data), &(output->json_data)))
  {
    return false;
  }
  return true;
}

astra_camera_msgs__msg__Metadata *
astra_camera_msgs__msg__Metadata__create()
{
  astra_camera_msgs__msg__Metadata * msg = (astra_camera_msgs__msg__Metadata *)malloc(sizeof(astra_camera_msgs__msg__Metadata));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astra_camera_msgs__msg__Metadata));
  bool success = astra_camera_msgs__msg__Metadata__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
astra_camera_msgs__msg__Metadata__destroy(astra_camera_msgs__msg__Metadata * msg)
{
  if (msg) {
    astra_camera_msgs__msg__Metadata__fini(msg);
  }
  free(msg);
}


bool
astra_camera_msgs__msg__Metadata__Sequence__init(astra_camera_msgs__msg__Metadata__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  astra_camera_msgs__msg__Metadata * data = NULL;
  if (size) {
    data = (astra_camera_msgs__msg__Metadata *)calloc(size, sizeof(astra_camera_msgs__msg__Metadata));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astra_camera_msgs__msg__Metadata__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astra_camera_msgs__msg__Metadata__fini(&data[i - 1]);
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
astra_camera_msgs__msg__Metadata__Sequence__fini(astra_camera_msgs__msg__Metadata__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      astra_camera_msgs__msg__Metadata__fini(&array->data[i]);
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

astra_camera_msgs__msg__Metadata__Sequence *
astra_camera_msgs__msg__Metadata__Sequence__create(size_t size)
{
  astra_camera_msgs__msg__Metadata__Sequence * array = (astra_camera_msgs__msg__Metadata__Sequence *)malloc(sizeof(astra_camera_msgs__msg__Metadata__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = astra_camera_msgs__msg__Metadata__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
astra_camera_msgs__msg__Metadata__Sequence__destroy(astra_camera_msgs__msg__Metadata__Sequence * array)
{
  if (array) {
    astra_camera_msgs__msg__Metadata__Sequence__fini(array);
  }
  free(array);
}

bool
astra_camera_msgs__msg__Metadata__Sequence__are_equal(const astra_camera_msgs__msg__Metadata__Sequence * lhs, const astra_camera_msgs__msg__Metadata__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astra_camera_msgs__msg__Metadata__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astra_camera_msgs__msg__Metadata__Sequence__copy(
  const astra_camera_msgs__msg__Metadata__Sequence * input,
  astra_camera_msgs__msg__Metadata__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astra_camera_msgs__msg__Metadata);
    astra_camera_msgs__msg__Metadata * data =
      (astra_camera_msgs__msg__Metadata *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astra_camera_msgs__msg__Metadata__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          astra_camera_msgs__msg__Metadata__fini(&data[i]);
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
    if (!astra_camera_msgs__msg__Metadata__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
