// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sum_srv_interface:srv/Sum.idl
// generated code does not contain a copyright notice

#ifndef SUM_SRV_INTERFACE__SRV__DETAIL__SUM__STRUCT_H_
#define SUM_SRV_INTERFACE__SRV__DETAIL__SUM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Sum in the package sum_srv_interface.
typedef struct sum_srv_interface__srv__Sum_Request
{
  int64_t a;
  int64_t b;
} sum_srv_interface__srv__Sum_Request;

// Struct for a sequence of sum_srv_interface__srv__Sum_Request.
typedef struct sum_srv_interface__srv__Sum_Request__Sequence
{
  sum_srv_interface__srv__Sum_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sum_srv_interface__srv__Sum_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Sum in the package sum_srv_interface.
typedef struct sum_srv_interface__srv__Sum_Response
{
  int64_t sum;
} sum_srv_interface__srv__Sum_Response;

// Struct for a sequence of sum_srv_interface__srv__Sum_Response.
typedef struct sum_srv_interface__srv__Sum_Response__Sequence
{
  sum_srv_interface__srv__Sum_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sum_srv_interface__srv__Sum_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SUM_SRV_INTERFACE__SRV__DETAIL__SUM__STRUCT_H_
