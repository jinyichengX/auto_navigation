// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sum_srv_interface:srv/Sum.idl
// generated code does not contain a copyright notice
#include "sum_srv_interface/srv/detail/sum__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
sum_srv_interface__srv__Sum_Request__init(sum_srv_interface__srv__Sum_Request * msg)
{
  if (!msg) {
    return false;
  }
  // a
  // b
  return true;
}

void
sum_srv_interface__srv__Sum_Request__fini(sum_srv_interface__srv__Sum_Request * msg)
{
  if (!msg) {
    return;
  }
  // a
  // b
}

bool
sum_srv_interface__srv__Sum_Request__are_equal(const sum_srv_interface__srv__Sum_Request * lhs, const sum_srv_interface__srv__Sum_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // a
  if (lhs->a != rhs->a) {
    return false;
  }
  // b
  if (lhs->b != rhs->b) {
    return false;
  }
  return true;
}

bool
sum_srv_interface__srv__Sum_Request__copy(
  const sum_srv_interface__srv__Sum_Request * input,
  sum_srv_interface__srv__Sum_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // a
  output->a = input->a;
  // b
  output->b = input->b;
  return true;
}

sum_srv_interface__srv__Sum_Request *
sum_srv_interface__srv__Sum_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sum_srv_interface__srv__Sum_Request * msg = (sum_srv_interface__srv__Sum_Request *)allocator.allocate(sizeof(sum_srv_interface__srv__Sum_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sum_srv_interface__srv__Sum_Request));
  bool success = sum_srv_interface__srv__Sum_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sum_srv_interface__srv__Sum_Request__destroy(sum_srv_interface__srv__Sum_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sum_srv_interface__srv__Sum_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sum_srv_interface__srv__Sum_Request__Sequence__init(sum_srv_interface__srv__Sum_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sum_srv_interface__srv__Sum_Request * data = NULL;

  if (size) {
    data = (sum_srv_interface__srv__Sum_Request *)allocator.zero_allocate(size, sizeof(sum_srv_interface__srv__Sum_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sum_srv_interface__srv__Sum_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sum_srv_interface__srv__Sum_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
sum_srv_interface__srv__Sum_Request__Sequence__fini(sum_srv_interface__srv__Sum_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      sum_srv_interface__srv__Sum_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

sum_srv_interface__srv__Sum_Request__Sequence *
sum_srv_interface__srv__Sum_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sum_srv_interface__srv__Sum_Request__Sequence * array = (sum_srv_interface__srv__Sum_Request__Sequence *)allocator.allocate(sizeof(sum_srv_interface__srv__Sum_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sum_srv_interface__srv__Sum_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sum_srv_interface__srv__Sum_Request__Sequence__destroy(sum_srv_interface__srv__Sum_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sum_srv_interface__srv__Sum_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sum_srv_interface__srv__Sum_Request__Sequence__are_equal(const sum_srv_interface__srv__Sum_Request__Sequence * lhs, const sum_srv_interface__srv__Sum_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sum_srv_interface__srv__Sum_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sum_srv_interface__srv__Sum_Request__Sequence__copy(
  const sum_srv_interface__srv__Sum_Request__Sequence * input,
  sum_srv_interface__srv__Sum_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sum_srv_interface__srv__Sum_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sum_srv_interface__srv__Sum_Request * data =
      (sum_srv_interface__srv__Sum_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sum_srv_interface__srv__Sum_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sum_srv_interface__srv__Sum_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sum_srv_interface__srv__Sum_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
sum_srv_interface__srv__Sum_Response__init(sum_srv_interface__srv__Sum_Response * msg)
{
  if (!msg) {
    return false;
  }
  // sum
  return true;
}

void
sum_srv_interface__srv__Sum_Response__fini(sum_srv_interface__srv__Sum_Response * msg)
{
  if (!msg) {
    return;
  }
  // sum
}

bool
sum_srv_interface__srv__Sum_Response__are_equal(const sum_srv_interface__srv__Sum_Response * lhs, const sum_srv_interface__srv__Sum_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sum
  if (lhs->sum != rhs->sum) {
    return false;
  }
  return true;
}

bool
sum_srv_interface__srv__Sum_Response__copy(
  const sum_srv_interface__srv__Sum_Response * input,
  sum_srv_interface__srv__Sum_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // sum
  output->sum = input->sum;
  return true;
}

sum_srv_interface__srv__Sum_Response *
sum_srv_interface__srv__Sum_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sum_srv_interface__srv__Sum_Response * msg = (sum_srv_interface__srv__Sum_Response *)allocator.allocate(sizeof(sum_srv_interface__srv__Sum_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sum_srv_interface__srv__Sum_Response));
  bool success = sum_srv_interface__srv__Sum_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sum_srv_interface__srv__Sum_Response__destroy(sum_srv_interface__srv__Sum_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sum_srv_interface__srv__Sum_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sum_srv_interface__srv__Sum_Response__Sequence__init(sum_srv_interface__srv__Sum_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sum_srv_interface__srv__Sum_Response * data = NULL;

  if (size) {
    data = (sum_srv_interface__srv__Sum_Response *)allocator.zero_allocate(size, sizeof(sum_srv_interface__srv__Sum_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sum_srv_interface__srv__Sum_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sum_srv_interface__srv__Sum_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
sum_srv_interface__srv__Sum_Response__Sequence__fini(sum_srv_interface__srv__Sum_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      sum_srv_interface__srv__Sum_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

sum_srv_interface__srv__Sum_Response__Sequence *
sum_srv_interface__srv__Sum_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sum_srv_interface__srv__Sum_Response__Sequence * array = (sum_srv_interface__srv__Sum_Response__Sequence *)allocator.allocate(sizeof(sum_srv_interface__srv__Sum_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sum_srv_interface__srv__Sum_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sum_srv_interface__srv__Sum_Response__Sequence__destroy(sum_srv_interface__srv__Sum_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sum_srv_interface__srv__Sum_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sum_srv_interface__srv__Sum_Response__Sequence__are_equal(const sum_srv_interface__srv__Sum_Response__Sequence * lhs, const sum_srv_interface__srv__Sum_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sum_srv_interface__srv__Sum_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sum_srv_interface__srv__Sum_Response__Sequence__copy(
  const sum_srv_interface__srv__Sum_Response__Sequence * input,
  sum_srv_interface__srv__Sum_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sum_srv_interface__srv__Sum_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sum_srv_interface__srv__Sum_Response * data =
      (sum_srv_interface__srv__Sum_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sum_srv_interface__srv__Sum_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sum_srv_interface__srv__Sum_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sum_srv_interface__srv__Sum_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
