// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sum_srv_interface:srv/Sum.idl
// generated code does not contain a copyright notice

#ifndef SUM_SRV_INTERFACE__SRV__DETAIL__SUM__BUILDER_HPP_
#define SUM_SRV_INTERFACE__SRV__DETAIL__SUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sum_srv_interface/srv/detail/sum__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sum_srv_interface
{

namespace srv
{

namespace builder
{

class Init_Sum_Request_b
{
public:
  explicit Init_Sum_Request_b(::sum_srv_interface::srv::Sum_Request & msg)
  : msg_(msg)
  {}
  ::sum_srv_interface::srv::Sum_Request b(::sum_srv_interface::srv::Sum_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sum_srv_interface::srv::Sum_Request msg_;
};

class Init_Sum_Request_a
{
public:
  Init_Sum_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sum_Request_b a(::sum_srv_interface::srv::Sum_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_Sum_Request_b(msg_);
  }

private:
  ::sum_srv_interface::srv::Sum_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sum_srv_interface::srv::Sum_Request>()
{
  return sum_srv_interface::srv::builder::Init_Sum_Request_a();
}

}  // namespace sum_srv_interface


namespace sum_srv_interface
{

namespace srv
{

namespace builder
{

class Init_Sum_Response_sum
{
public:
  Init_Sum_Response_sum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sum_srv_interface::srv::Sum_Response sum(::sum_srv_interface::srv::Sum_Response::_sum_type arg)
  {
    msg_.sum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sum_srv_interface::srv::Sum_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sum_srv_interface::srv::Sum_Response>()
{
  return sum_srv_interface::srv::builder::Init_Sum_Response_sum();
}

}  // namespace sum_srv_interface

#endif  // SUM_SRV_INTERFACE__SRV__DETAIL__SUM__BUILDER_HPP_
