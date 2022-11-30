// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from beginner_tutorials:srv/UpdateMessage.idl
// generated code does not contain a copyright notice

#ifndef BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__BUILDER_HPP_
#define BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "beginner_tutorials/srv/detail/update_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace beginner_tutorials
{

namespace srv
{

namespace builder
{

class Init_UpdateMessage_Request_data
{
public:
  Init_UpdateMessage_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::beginner_tutorials::srv::UpdateMessage_Request data(::beginner_tutorials::srv::UpdateMessage_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::beginner_tutorials::srv::UpdateMessage_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::beginner_tutorials::srv::UpdateMessage_Request>()
{
  return beginner_tutorials::srv::builder::Init_UpdateMessage_Request_data();
}

}  // namespace beginner_tutorials


namespace beginner_tutorials
{

namespace srv
{

namespace builder
{

class Init_UpdateMessage_Response_new_message
{
public:
  Init_UpdateMessage_Response_new_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::beginner_tutorials::srv::UpdateMessage_Response new_message(::beginner_tutorials::srv::UpdateMessage_Response::_new_message_type arg)
  {
    msg_.new_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::beginner_tutorials::srv::UpdateMessage_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::beginner_tutorials::srv::UpdateMessage_Response>()
{
  return beginner_tutorials::srv::builder::Init_UpdateMessage_Response_new_message();
}

}  // namespace beginner_tutorials

#endif  // BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__BUILDER_HPP_
