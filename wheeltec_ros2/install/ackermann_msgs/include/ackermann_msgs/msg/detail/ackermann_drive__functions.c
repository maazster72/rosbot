// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ackermann_msgs:msg/AckermannDrive.idl
// generated code does not contain a copyright notice
#include "ackermann_msgs/msg/detail/ackermann_drive__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
ackermann_msgs__msg__AckermannDrive__init(ackermann_msgs__msg__AckermannDrive * msg)
{
  if (!msg) {
    return false;
  }
  // steering_angle
  // steering_angle_velocity
  // speed
  // acceleration
  // jerk
  return true;
}

void
ackermann_msgs__msg__AckermannDrive__fini(ackermann_msgs__msg__AckermannDrive * msg)
{
  if (!msg) {
    return;
  }
  // steering_angle
  // steering_angle_velocity
  // speed
  // acceleration
  // jerk
}

bool
ackermann_msgs__msg__AckermannDrive__are_equal(const ackermann_msgs__msg__AckermannDrive * lhs, const ackermann_msgs__msg__AckermannDrive * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // steering_angle
  if (lhs->steering_angle != rhs->steering_angle) {
    return false;
  }
  // steering_angle_velocity
  if (lhs->steering_angle_velocity != rhs->steering_angle_velocity) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // acceleration
  if (lhs->acceleration != rhs->acceleration) {
    return false;
  }
  // jerk
  if (lhs->jerk != rhs->jerk) {
    return false;
  }
  return true;
}

bool
ackermann_msgs__msg__AckermannDrive__copy(
  const ackermann_msgs__msg__AckermannDrive * input,
  ackermann_msgs__msg__AckermannDrive * output)
{
  if (!input || !output) {
    return false;
  }
  // steering_angle
  output->steering_angle = input->steering_angle;
  // steering_angle_velocity
  output->steering_angle_velocity = input->steering_angle_velocity;
  // speed
  output->speed = input->speed;
  // acceleration
  output->acceleration = input->acceleration;
  // jerk
  output->jerk = input->jerk;
  return true;
}

ackermann_msgs__msg__AckermannDrive *
ackermann_msgs__msg__AckermannDrive__create()
{
  ackermann_msgs__msg__AckermannDrive * msg = (ackermann_msgs__msg__AckermannDrive *)malloc(sizeof(ackermann_msgs__msg__AckermannDrive));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ackermann_msgs__msg__AckermannDrive));
  bool success = ackermann_msgs__msg__AckermannDrive__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
ackermann_msgs__msg__AckermannDrive__destroy(ackermann_msgs__msg__AckermannDrive * msg)
{
  if (msg) {
    ackermann_msgs__msg__AckermannDrive__fini(msg);
  }
  free(msg);
}


bool
ackermann_msgs__msg__AckermannDrive__Sequence__init(ackermann_msgs__msg__AckermannDrive__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  ackermann_msgs__msg__AckermannDrive * data = NULL;
  if (size) {
    data = (ackermann_msgs__msg__AckermannDrive *)calloc(size, sizeof(ackermann_msgs__msg__AckermannDrive));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ackermann_msgs__msg__AckermannDrive__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ackermann_msgs__msg__AckermannDrive__fini(&data[i - 1]);
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
ackermann_msgs__msg__AckermannDrive__Sequence__fini(ackermann_msgs__msg__AckermannDrive__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ackermann_msgs__msg__AckermannDrive__fini(&array->data[i]);
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

ackermann_msgs__msg__AckermannDrive__Sequence *
ackermann_msgs__msg__AckermannDrive__Sequence__create(size_t size)
{
  ackermann_msgs__msg__AckermannDrive__Sequence * array = (ackermann_msgs__msg__AckermannDrive__Sequence *)malloc(sizeof(ackermann_msgs__msg__AckermannDrive__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = ackermann_msgs__msg__AckermannDrive__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
ackermann_msgs__msg__AckermannDrive__Sequence__destroy(ackermann_msgs__msg__AckermannDrive__Sequence * array)
{
  if (array) {
    ackermann_msgs__msg__AckermannDrive__Sequence__fini(array);
  }
  free(array);
}

bool
ackermann_msgs__msg__AckermannDrive__Sequence__are_equal(const ackermann_msgs__msg__AckermannDrive__Sequence * lhs, const ackermann_msgs__msg__AckermannDrive__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ackermann_msgs__msg__AckermannDrive__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ackermann_msgs__msg__AckermannDrive__Sequence__copy(
  const ackermann_msgs__msg__AckermannDrive__Sequence * input,
  ackermann_msgs__msg__AckermannDrive__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ackermann_msgs__msg__AckermannDrive);
    ackermann_msgs__msg__AckermannDrive * data =
      (ackermann_msgs__msg__AckermannDrive *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ackermann_msgs__msg__AckermannDrive__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ackermann_msgs__msg__AckermannDrive__fini(&data[i]);
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
    if (!ackermann_msgs__msg__AckermannDrive__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
