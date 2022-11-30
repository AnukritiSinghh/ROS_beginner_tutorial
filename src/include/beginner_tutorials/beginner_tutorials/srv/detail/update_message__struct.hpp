// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from beginner_tutorials:srv/UpdateMessage.idl
// generated code does not contain a copyright notice

#ifndef BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__STRUCT_HPP_
#define BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__beginner_tutorials__srv__UpdateMessage_Request __attribute__((deprecated))
#else
# define DEPRECATED__beginner_tutorials__srv__UpdateMessage_Request __declspec(deprecated)
#endif

namespace beginner_tutorials
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UpdateMessage_Request_
{
  using Type = UpdateMessage_Request_<ContainerAllocator>;

  explicit UpdateMessage_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  explicit UpdateMessage_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__beginner_tutorials__srv__UpdateMessage_Request
    std::shared_ptr<beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__beginner_tutorials__srv__UpdateMessage_Request
    std::shared_ptr<beginner_tutorials::srv::UpdateMessage_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateMessage_Request_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateMessage_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateMessage_Request_

// alias to use template instance with default allocator
using UpdateMessage_Request =
  beginner_tutorials::srv::UpdateMessage_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace beginner_tutorials


#ifndef _WIN32
# define DEPRECATED__beginner_tutorials__srv__UpdateMessage_Response __attribute__((deprecated))
#else
# define DEPRECATED__beginner_tutorials__srv__UpdateMessage_Response __declspec(deprecated)
#endif

namespace beginner_tutorials
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UpdateMessage_Response_
{
  using Type = UpdateMessage_Response_<ContainerAllocator>;

  explicit UpdateMessage_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->new_message = "";
    }
  }

  explicit UpdateMessage_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : new_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->new_message = "";
    }
  }

  // field types and members
  using _new_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _new_message_type new_message;

  // setters for named parameter idiom
  Type & set__new_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->new_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__beginner_tutorials__srv__UpdateMessage_Response
    std::shared_ptr<beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__beginner_tutorials__srv__UpdateMessage_Response
    std::shared_ptr<beginner_tutorials::srv::UpdateMessage_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateMessage_Response_ & other) const
  {
    if (this->new_message != other.new_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateMessage_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateMessage_Response_

// alias to use template instance with default allocator
using UpdateMessage_Response =
  beginner_tutorials::srv::UpdateMessage_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace beginner_tutorials

namespace beginner_tutorials
{

namespace srv
{

struct UpdateMessage
{
  using Request = beginner_tutorials::srv::UpdateMessage_Request;
  using Response = beginner_tutorials::srv::UpdateMessage_Response;
};

}  // namespace srv

}  // namespace beginner_tutorials

#endif  // BEGINNER_TUTORIALS__SRV__DETAIL__UPDATE_MESSAGE__STRUCT_HPP_
