// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astra_camera_msgs:msg/Extrinsics.idl
// generated code does not contain a copyright notice
#include "astra_camera_msgs/msg/detail/extrinsics__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
astra_camera_msgs__msg__Extrinsics__init(astra_camera_msgs__msg__Extrinsics * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    astra_camera_msgs__msg__Extrinsics__fini(msg);
    return false;
  }
  // rotation
  // translation
  return true;
}

void
astra_camera_msgs__msg__Extrinsics__fini(astra_camera_msgs__msg__Extrinsics * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // rotation
  // translation
}

bool
astra_camera_msgs__msg__Extrinsics__are_equal(const astra_camera_msgs__msg__Extrinsics * lhs, const astra_camera_msgs__msg__Extrinsics * rhs)
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
  // rotation
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->rotation[i] != rhs->rotation[i]) {
      return false;
    }
  }
  // translation
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->translation[i] != rhs->translation[i]) {
      return false;
    }
  }
  return true;
}

bool
astra_camera_msgs__msg__Extrinsics__copy(
  const astra_camera_msgs__msg__Extrinsics * input,
  astra_camera_msgs__msg__Extrinsics * output)
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
  // rotation
  for (size_t i = 0; i < 9; ++i) {
    output->rotation[i] = input->rotation[i];
  }
  // translation
  for (size_t i = 0; i < 3; ++i) {
    output->translation[i] = input->translation[i];
  }
  return true;
}

astra_camera_msgs__msg__Extrinsics *
astra_camera_msgs__msg__Extrinsics__create()
{
  astra_camera_msgs__msg__Extrinsics * msg = (astra_camera_msgs__msg__Extrinsics *)malloc(sizeof(astra_camera_msgs__msg__Extrinsics));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astra_camera_msgs__msg__Extrinsics));
  bool success = astra_camera_msgs__msg__Extrinsics__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
astra_camera_msgs__msg__Extrinsics__destroy(astra_camera_msgs__msg__Extrinsics * msg)
{
  if (msg) {
    astra_camera_msgs__msg__Extrinsics__fini(msg);
  }
  free(msg);
}


bool
astra_camera_msgs__msg__Extrinsics__Sequence__init(astra_camera_msgs__msg__Extrinsics__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  astra_camera_msgs__msg__Extrinsics * data = NULL;
  if (size) {
    data = (astra_camera_msgs__msg__Extrinsics *)calloc(size, sizeof(astra_camera_msgs__msg__Extrinsics));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astra_camera_msgs__msg__Extrinsics__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astra_camera_msgs__msg__Extrinsics__fini(&data[i - 1]);
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
astra_camera_msgs__msg__Extrinsics__Sequence__fini(astra_camera_msgs__msg__Extrinsics__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      astra_camera_msgs__msg__Extrinsics__fini(&array->data[i]);
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

astra_camera_msgs__msg__Extrinsics__Sequence *
astra_camera_msgs__msg__Extrinsics__Sequence__create(size_t size)
{
  astra_camera_msgs__msg__Extrinsics__Sequence * array = (astra_camera_msgs__msg__Extrinsics__Sequence *)malloc(sizeof(astra_camera_msgs__msg__Extrinsics__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = astra_camera_msgs__msg__Extrinsics__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
astra_camera_msgs__msg__Extrinsics__Sequence__destroy(astra_camera_msgs__msg__Extrinsics__Sequence * array)
{
  if (array) {
    astra_camera_msgs__msg__Extrinsics__Sequence__fini(array);
  }
  free(array);
}

bool
astra_camera_msgs__msg__Extrinsics__Sequence__are_equal(const astra_camera_msgs__msg__Extrinsics__Sequence * lhs, const astra_camera_msgs__msg__Extrinsics__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astra_camera_msgs__msg__Extrinsics__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astra_camera_msgs__msg__Extrinsics__Sequence__copy(
  const astra_camera_msgs__msg__Extrinsics__Sequence * input,
  astra_camera_msgs__msg__Extrinsics__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astra_camera_msgs__msg__Extrinsics);
    astra_camera_msgs__msg__Extrinsics * data =
      (astra_camera_msgs__msg__Extrinsics *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astra_camera_msgs__msg__Extrinsics__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          astra_camera_msgs__msg__Extrinsics__fini(&data[i]);
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
    if (!astra_camera_msgs__msg__Extrinsics__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
