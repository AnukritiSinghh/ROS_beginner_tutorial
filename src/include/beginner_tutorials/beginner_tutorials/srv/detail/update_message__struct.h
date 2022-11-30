// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from beginner_tutorials:srv/UpdateMessage.idl
// generated code does not contain a copyright notice

#ifndef BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__STRUCT_H_
#define BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/UpdateMessage in the package beginner_tutorials.
typedef struct beginner_tutorials__srv__UpdateMessage_Request
{
  rosidl_runtime_c__String data;
} beginner_tutorials__srv__UpdateMessage_Request;

// Struct for a sequence of beginner_tutorials__srv__UpdateMessage_Request.
typedef struct beginner_tutorials__srv__UpdateMessage_Request__Sequence
{
  beginner_tutorials__srv__UpdateMessage_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} beginner_tutorials__srv__UpdateMessage_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'new_message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/UpdateMessage in the package beginner_tutorials.
typedef struct beginner_tutorials__srv__UpdateMessage_Response
{
  rosidl_runtime_c__String new_message;
} beginner_tutorials__srv__UpdateMessage_Response;

// Struct for a sequence of beginner_tutorials__srv__UpdateMessage_Response.
typedef struct beginner_tutorials__srv__UpdateMessage_Response__Sequence
{
  beginner_tutorials__srv__UpdateMessage_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} beginner_tutorials__srv__UpdateMessage_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__STRUCT_H_
