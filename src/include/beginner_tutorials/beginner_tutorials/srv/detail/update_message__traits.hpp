// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from beginner_tutorials:srv/UpdateMessage.idl
// generated code does not contain a copyright notice

#ifndef BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__TRAITS_HPP_
#define BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "beginner_tutorials/srv/detail/update_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace beginner_tutorials
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateMessage_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpdateMessage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateMessage_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace beginner_tutorials

namespace rosidl_generator_traits
{

[[deprecated("use beginner_tutorials::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const beginner_tutorials::srv::UpdateMessage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  beginner_tutorials::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use beginner_tutorials::srv::to_yaml() instead")]]
inline std::string to_yaml(const beginner_tutorials::srv::UpdateMessage_Request & msg)
{
  return beginner_tutorials::srv::to_yaml(msg);
}

template<>
inline const char * data_type<beginner_tutorials::srv::UpdateMessage_Request>()
{
  return "beginner_tutorials::srv::UpdateMessage_Request";
}

template<>
inline const char * name<beginner_tutorials::srv::UpdateMessage_Request>()
{
  return "beginner_tutorials/srv/UpdateMessage_Request";
}

template<>
struct has_fixed_size<beginner_tutorials::srv::UpdateMessage_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<beginner_tutorials::srv::UpdateMessage_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<beginner_tutorials::srv::UpdateMessage_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace beginner_tutorials
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateMessage_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: new_message
  {
    out << "new_message: ";
    rosidl_generator_traits::value_to_yaml(msg.new_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpdateMessage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: new_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "new_message: ";
    rosidl_generator_traits::value_to_yaml(msg.new_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateMessage_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace beginner_tutorials

namespace rosidl_generator_traits
{

[[deprecated("use beginner_tutorials::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const beginner_tutorials::srv::UpdateMessage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  beginner_tutorials::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use beginner_tutorials::srv::to_yaml() instead")]]
inline std::string to_yaml(const beginner_tutorials::srv::UpdateMessage_Response & msg)
{
  return beginner_tutorials::srv::to_yaml(msg);
}

template<>
inline const char * data_type<beginner_tutorials::srv::UpdateMessage_Response>()
{
  return "beginner_tutorials::srv::UpdateMessage_Response";
}

template<>
inline const char * name<beginner_tutorials::srv::UpdateMessage_Response>()
{
  return "beginner_tutorials/srv/UpdateMessage_Response";
}

template<>
struct has_fixed_size<beginner_tutorials::srv::UpdateMessage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<beginner_tutorials::srv::UpdateMessage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<beginner_tutorials::srv::UpdateMessage_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<beginner_tutorials::srv::UpdateMessage>()
{
  return "beginner_tutorials::srv::UpdateMessage";
}

template<>
inline const char * name<beginner_tutorials::srv::UpdateMessage>()
{
  return "beginner_tutorials/srv/UpdateMessage";
}

template<>
struct has_fixed_size<beginner_tutorials::srv::UpdateMessage>
  : std::integral_constant<
    bool,
    has_fixed_size<beginner_tutorials::srv::UpdateMessage_Request>::value &&
    has_fixed_size<beginner_tutorials::srv::UpdateMessage_Response>::value
  >
{
};

template<>
struct has_bounded_size<beginner_tutorials::srv::UpdateMessage>
  : std::integral_constant<
    bool,
    has_bounded_size<beginner_tutorials::srv::UpdateMessage_Request>::value &&
    has_bounded_size<beginner_tutorials::srv::UpdateMessage_Response>::value
  >
{
};

template<>
struct is_service<beginner_tutorials::srv::UpdateMessage>
  : std::true_type
{
};

template<>
struct is_service_request<beginner_tutorials::srv::UpdateMessage_Request>
  : std::true_type
{
};

template<>
struct is_service_response<beginner_tutorials::srv::UpdateMessage_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__TRAITS_HPP_
