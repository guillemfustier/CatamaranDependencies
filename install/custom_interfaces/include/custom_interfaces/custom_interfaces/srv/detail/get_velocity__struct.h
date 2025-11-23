// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:srv/GetVelocity.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__GET_VELOCITY__STRUCT_H_
#define CUSTOM_INTERFACES__SRV__DETAIL__GET_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetVelocity in the package custom_interfaces.
typedef struct custom_interfaces__srv__GetVelocity_Request
{
  uint8_t id;
} custom_interfaces__srv__GetVelocity_Request;

// Struct for a sequence of custom_interfaces__srv__GetVelocity_Request.
typedef struct custom_interfaces__srv__GetVelocity_Request__Sequence
{
  custom_interfaces__srv__GetVelocity_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__GetVelocity_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GetVelocity in the package custom_interfaces.
typedef struct custom_interfaces__srv__GetVelocity_Response
{
  int32_t velocity;
} custom_interfaces__srv__GetVelocity_Response;

// Struct for a sequence of custom_interfaces__srv__GetVelocity_Response.
typedef struct custom_interfaces__srv__GetVelocity_Response__Sequence
{
  custom_interfaces__srv__GetVelocity_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__GetVelocity_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__GET_VELOCITY__STRUCT_H_
