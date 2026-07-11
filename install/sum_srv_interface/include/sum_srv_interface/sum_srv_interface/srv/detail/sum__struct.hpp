// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sum_srv_interface:srv/Sum.idl
// generated code does not contain a copyright notice

#ifndef SUM_SRV_INTERFACE__SRV__DETAIL__SUM__STRUCT_HPP_
#define SUM_SRV_INTERFACE__SRV__DETAIL__SUM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sum_srv_interface__srv__Sum_Request __attribute__((deprecated))
#else
# define DEPRECATED__sum_srv_interface__srv__Sum_Request __declspec(deprecated)
#endif

namespace sum_srv_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Sum_Request_
{
  using Type = Sum_Request_<ContainerAllocator>;

  explicit Sum_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0ll;
      this->b = 0ll;
    }
  }

  explicit Sum_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0ll;
      this->b = 0ll;
    }
  }

  // field types and members
  using _a_type =
    int64_t;
  _a_type a;
  using _b_type =
    int64_t;
  _b_type b;

  // setters for named parameter idiom
  Type & set__a(
    const int64_t & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const int64_t & _arg)
  {
    this->b = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sum_srv_interface::srv::Sum_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const sum_srv_interface::srv::Sum_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sum_srv_interface::srv::Sum_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sum_srv_interface::srv::Sum_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sum_srv_interface::srv::Sum_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sum_srv_interface::srv::Sum_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sum_srv_interface::srv::Sum_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sum_srv_interface::srv::Sum_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sum_srv_interface::srv::Sum_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sum_srv_interface::srv::Sum_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sum_srv_interface__srv__Sum_Request
    std::shared_ptr<sum_srv_interface::srv::Sum_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sum_srv_interface__srv__Sum_Request
    std::shared_ptr<sum_srv_interface::srv::Sum_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sum_Request_ & other) const
  {
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sum_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sum_Request_

// alias to use template instance with default allocator
using Sum_Request =
  sum_srv_interface::srv::Sum_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace sum_srv_interface


#ifndef _WIN32
# define DEPRECATED__sum_srv_interface__srv__Sum_Response __attribute__((deprecated))
#else
# define DEPRECATED__sum_srv_interface__srv__Sum_Response __declspec(deprecated)
#endif

namespace sum_srv_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Sum_Response_
{
  using Type = Sum_Response_<ContainerAllocator>;

  explicit Sum_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sum = 0ll;
    }
  }

  explicit Sum_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sum = 0ll;
    }
  }

  // field types and members
  using _sum_type =
    int64_t;
  _sum_type sum;

  // setters for named parameter idiom
  Type & set__sum(
    const int64_t & _arg)
  {
    this->sum = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sum_srv_interface::srv::Sum_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const sum_srv_interface::srv::Sum_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sum_srv_interface::srv::Sum_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sum_srv_interface::srv::Sum_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sum_srv_interface::srv::Sum_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sum_srv_interface::srv::Sum_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sum_srv_interface::srv::Sum_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sum_srv_interface::srv::Sum_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sum_srv_interface::srv::Sum_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sum_srv_interface::srv::Sum_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sum_srv_interface__srv__Sum_Response
    std::shared_ptr<sum_srv_interface::srv::Sum_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sum_srv_interface__srv__Sum_Response
    std::shared_ptr<sum_srv_interface::srv::Sum_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sum_Response_ & other) const
  {
    if (this->sum != other.sum) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sum_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sum_Response_

// alias to use template instance with default allocator
using Sum_Response =
  sum_srv_interface::srv::Sum_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace sum_srv_interface

namespace sum_srv_interface
{

namespace srv
{

struct Sum
{
  using Request = sum_srv_interface::srv::Sum_Request;
  using Response = sum_srv_interface::srv::Sum_Response;
};

}  // namespace srv

}  // namespace sum_srv_interface

#endif  // SUM_SRV_INTERFACE__SRV__DETAIL__SUM__STRUCT_HPP_
