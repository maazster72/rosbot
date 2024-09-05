// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dwb_msgs:msg/TrajectoryScore.idl
// generated code does not contain a copyright notice
#include "dwb_msgs/msg/detail/trajectory_score__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `traj`
#include "dwb_msgs/msg/detail/trajectory2_d__functions.h"
// Member `scores`
#include "dwb_msgs/msg/detail/critic_score__functions.h"

bool
dwb_msgs__msg__TrajectoryScore__init(dwb_msgs__msg__TrajectoryScore * msg)
{
  if (!msg) {
    return false;
  }
  // traj
  if (!dwb_msgs__msg__Trajectory2D__init(&msg->traj)) {
    dwb_msgs__msg__TrajectoryScore__fini(msg);
    return false;
  }
  // scores
  if (!dwb_msgs__msg__CriticScore__Sequence__init(&msg->scores, 0)) {
    dwb_msgs__msg__TrajectoryScore__fini(msg);
    return false;
  }
  // total
  return true;
}

void
dwb_msgs__msg__TrajectoryScore__fini(dwb_msgs__msg__TrajectoryScore * msg)
{
  if (!msg) {
    return;
  }
  // traj
  dwb_msgs__msg__Trajectory2D__fini(&msg->traj);
  // scores
  dwb_msgs__msg__CriticScore__Sequence__fini(&msg->scores);
  // total
}

bool
dwb_msgs__msg__TrajectoryScore__are_equal(const dwb_msgs__msg__TrajectoryScore * lhs, const dwb_msgs__msg__TrajectoryScore * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // traj
  if (!dwb_msgs__msg__Trajectory2D__are_equal(
      &(lhs->traj), &(rhs->traj)))
  {
    return false;
  }
  // scores
  if (!dwb_msgs__msg__CriticScore__Sequence__are_equal(
      &(lhs->scores), &(rhs->scores)))
  {
    return false;
  }
  // total
  if (lhs->total != rhs->total) {
    return false;
  }
  return true;
}

bool
dwb_msgs__msg__TrajectoryScore__copy(
  const dwb_msgs__msg__TrajectoryScore * input,
  dwb_msgs__msg__TrajectoryScore * output)
{
  if (!input || !output) {
    return false;
  }
  // traj
  if (!dwb_msgs__msg__Trajectory2D__copy(
      &(input->traj), &(output->traj)))
  {
    return false;
  }
  // scores
  if (!dwb_msgs__msg__CriticScore__Sequence__copy(
      &(input->scores), &(output->scores)))
  {
    return false;
  }
  // total
  output->total = input->total;
  return true;
}

dwb_msgs__msg__TrajectoryScore *
dwb_msgs__msg__TrajectoryScore__create()
{
  dwb_msgs__msg__TrajectoryScore * msg = (dwb_msgs__msg__TrajectoryScore *)malloc(sizeof(dwb_msgs__msg__TrajectoryScore));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dwb_msgs__msg__TrajectoryScore));
  bool success = dwb_msgs__msg__TrajectoryScore__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
dwb_msgs__msg__TrajectoryScore__destroy(dwb_msgs__msg__TrajectoryScore * msg)
{
  if (msg) {
    dwb_msgs__msg__TrajectoryScore__fini(msg);
  }
  free(msg);
}


bool
dwb_msgs__msg__TrajectoryScore__Sequence__init(dwb_msgs__msg__TrajectoryScore__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  dwb_msgs__msg__TrajectoryScore * data = NULL;
  if (size) {
    data = (dwb_msgs__msg__TrajectoryScore *)calloc(size, sizeof(dwb_msgs__msg__TrajectoryScore));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dwb_msgs__msg__TrajectoryScore__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dwb_msgs__msg__TrajectoryScore__fini(&data[i - 1]);
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
dwb_msgs__msg__TrajectoryScore__Sequence__fini(dwb_msgs__msg__TrajectoryScore__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dwb_msgs__msg__TrajectoryScore__fini(&array->data[i]);
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

dwb_msgs__msg__TrajectoryScore__Sequence *
dwb_msgs__msg__TrajectoryScore__Sequence__create(size_t size)
{
  dwb_msgs__msg__TrajectoryScore__Sequence * array = (dwb_msgs__msg__TrajectoryScore__Sequence *)malloc(sizeof(dwb_msgs__msg__TrajectoryScore__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = dwb_msgs__msg__TrajectoryScore__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
dwb_msgs__msg__TrajectoryScore__Sequence__destroy(dwb_msgs__msg__TrajectoryScore__Sequence * array)
{
  if (array) {
    dwb_msgs__msg__TrajectoryScore__Sequence__fini(array);
  }
  free(array);
}

bool
dwb_msgs__msg__TrajectoryScore__Sequence__are_equal(const dwb_msgs__msg__TrajectoryScore__Sequence * lhs, const dwb_msgs__msg__TrajectoryScore__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dwb_msgs__msg__TrajectoryScore__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dwb_msgs__msg__TrajectoryScore__Sequence__copy(
  const dwb_msgs__msg__TrajectoryScore__Sequence * input,
  dwb_msgs__msg__TrajectoryScore__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dwb_msgs__msg__TrajectoryScore);
    dwb_msgs__msg__TrajectoryScore * data =
      (dwb_msgs__msg__TrajectoryScore *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dwb_msgs__msg__TrajectoryScore__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dwb_msgs__msg__TrajectoryScore__fini(&data[i]);
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
    if (!dwb_msgs__msg__TrajectoryScore__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
