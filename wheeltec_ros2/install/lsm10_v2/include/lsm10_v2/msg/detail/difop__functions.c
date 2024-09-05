// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lsm10_v2:msg/Difop.idl
// generated code does not contain a copyright notice
#include "lsm10_v2/msg/detail/difop__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
lsm10_v2__msg__Difop__init(lsm10_v2__msg__Difop * msg)
{
  if (!msg) {
    return false;
  }
  // motorspeed
  return true;
}

void
lsm10_v2__msg__Difop__fini(lsm10_v2__msg__Difop * msg)
{
  if (!msg) {
    return;
  }
  // motorspeed
}

bool
lsm10_v2__msg__Difop__are_equal(const lsm10_v2__msg__Difop * lhs, const lsm10_v2__msg__Difop * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motorspeed
  if (lhs->motorspeed != rhs->motorspeed) {
    return false;
  }
  return true;
}

bool
lsm10_v2__msg__Difop__copy(
  const lsm10_v2__msg__Difop * input,
  lsm10_v2__msg__Difop * output)
{
  if (!input || !output) {
    return false;
  }
  // motorspeed
  output->motorspeed = input->motorspeed;
  return true;
}

lsm10_v2__msg__Difop *
lsm10_v2__msg__Difop__create()
{
  lsm10_v2__msg__Difop * msg = (lsm10_v2__msg__Difop *)malloc(sizeof(lsm10_v2__msg__Difop));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lsm10_v2__msg__Difop));
  bool success = lsm10_v2__msg__Difop__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
lsm10_v2__msg__Difop__destroy(lsm10_v2__msg__Difop * msg)
{
  if (msg) {
    lsm10_v2__msg__Difop__fini(msg);
  }
  free(msg);
}


bool
lsm10_v2__msg__Difop__Sequence__init(lsm10_v2__msg__Difop__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  lsm10_v2__msg__Difop * data = NULL;
  if (size) {
    data = (lsm10_v2__msg__Difop *)calloc(size, sizeof(lsm10_v2__msg__Difop));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lsm10_v2__msg__Difop__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lsm10_v2__msg__Difop__fini(&data[i - 1]);
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
lsm10_v2__msg__Difop__Sequence__fini(lsm10_v2__msg__Difop__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      lsm10_v2__msg__Difop__fini(&array->data[i]);
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

lsm10_v2__msg__Difop__Sequence *
lsm10_v2__msg__Difop__Sequence__create(size_t size)
{
  lsm10_v2__msg__Difop__Sequence * array = (lsm10_v2__msg__Difop__Sequence *)malloc(sizeof(lsm10_v2__msg__Difop__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = lsm10_v2__msg__Difop__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
lsm10_v2__msg__Difop__Sequence__destroy(lsm10_v2__msg__Difop__Sequence * array)
{
  if (array) {
    lsm10_v2__msg__Difop__Sequence__fini(array);
  }
  free(array);
}

bool
lsm10_v2__msg__Difop__Sequence__are_equal(const lsm10_v2__msg__Difop__Sequence * lhs, const lsm10_v2__msg__Difop__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lsm10_v2__msg__Difop__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lsm10_v2__msg__Difop__Sequence__copy(
  const lsm10_v2__msg__Difop__Sequence * input,
  lsm10_v2__msg__Difop__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lsm10_v2__msg__Difop);
    lsm10_v2__msg__Difop * data =
      (lsm10_v2__msg__Difop *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lsm10_v2__msg__Difop__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          lsm10_v2__msg__Difop__fini(&data[i]);
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
    if (!lsm10_v2__msg__Difop__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
