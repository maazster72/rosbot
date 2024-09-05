// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nav_2d_msgs:msg/Path2D.idl
// generated code does not contain a copyright notice
#include "nav_2d_msgs/msg/detail/path2_d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `poses`
#include "geometry_msgs/msg/detail/pose2_d__functions.h"

bool
nav_2d_msgs__msg__Path2D__init(nav_2d_msgs__msg__Path2D * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    nav_2d_msgs__msg__Path2D__fini(msg);
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose2D__Sequence__init(&msg->poses, 0)) {
    nav_2d_msgs__msg__Path2D__fini(msg);
    return false;
  }
  return true;
}

void
nav_2d_msgs__msg__Path2D__fini(nav_2d_msgs__msg__Path2D * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // poses
  geometry_msgs__msg__Pose2D__Sequence__fini(&msg->poses);
}

bool
nav_2d_msgs__msg__Path2D__are_equal(const nav_2d_msgs__msg__Path2D * lhs, const nav_2d_msgs__msg__Path2D * rhs)
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
  // poses
  if (!geometry_msgs__msg__Pose2D__Sequence__are_equal(
      &(lhs->poses), &(rhs->poses)))
  {
    return false;
  }
  return true;
}

bool
nav_2d_msgs__msg__Path2D__copy(
  const nav_2d_msgs__msg__Path2D * input,
  nav_2d_msgs__msg__Path2D * output)
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
  // poses
  if (!geometry_msgs__msg__Pose2D__Sequence__copy(
      &(input->poses), &(output->poses)))
  {
    return false;
  }
  return true;
}

nav_2d_msgs__msg__Path2D *
nav_2d_msgs__msg__Path2D__create()
{
  nav_2d_msgs__msg__Path2D * msg = (nav_2d_msgs__msg__Path2D *)malloc(sizeof(nav_2d_msgs__msg__Path2D));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav_2d_msgs__msg__Path2D));
  bool success = nav_2d_msgs__msg__Path2D__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
nav_2d_msgs__msg__Path2D__destroy(nav_2d_msgs__msg__Path2D * msg)
{
  if (msg) {
    nav_2d_msgs__msg__Path2D__fini(msg);
  }
  free(msg);
}


bool
nav_2d_msgs__msg__Path2D__Sequence__init(nav_2d_msgs__msg__Path2D__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  nav_2d_msgs__msg__Path2D * data = NULL;
  if (size) {
    data = (nav_2d_msgs__msg__Path2D *)calloc(size, sizeof(nav_2d_msgs__msg__Path2D));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav_2d_msgs__msg__Path2D__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav_2d_msgs__msg__Path2D__fini(&data[i - 1]);
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
nav_2d_msgs__msg__Path2D__Sequence__fini(nav_2d_msgs__msg__Path2D__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav_2d_msgs__msg__Path2D__fini(&array->data[i]);
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

nav_2d_msgs__msg__Path2D__Sequence *
nav_2d_msgs__msg__Path2D__Sequence__create(size_t size)
{
  nav_2d_msgs__msg__Path2D__Sequence * array = (nav_2d_msgs__msg__Path2D__Sequence *)malloc(sizeof(nav_2d_msgs__msg__Path2D__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = nav_2d_msgs__msg__Path2D__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
nav_2d_msgs__msg__Path2D__Sequence__destroy(nav_2d_msgs__msg__Path2D__Sequence * array)
{
  if (array) {
    nav_2d_msgs__msg__Path2D__Sequence__fini(array);
  }
  free(array);
}

bool
nav_2d_msgs__msg__Path2D__Sequence__are_equal(const nav_2d_msgs__msg__Path2D__Sequence * lhs, const nav_2d_msgs__msg__Path2D__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nav_2d_msgs__msg__Path2D__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nav_2d_msgs__msg__Path2D__Sequence__copy(
  const nav_2d_msgs__msg__Path2D__Sequence * input,
  nav_2d_msgs__msg__Path2D__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nav_2d_msgs__msg__Path2D);
    nav_2d_msgs__msg__Path2D * data =
      (nav_2d_msgs__msg__Path2D *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nav_2d_msgs__msg__Path2D__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          nav_2d_msgs__msg__Path2D__fini(&data[i]);
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
    if (!nav_2d_msgs__msg__Path2D__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
