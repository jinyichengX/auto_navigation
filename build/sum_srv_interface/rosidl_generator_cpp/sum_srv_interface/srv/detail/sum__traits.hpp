// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sum_srv_interface:srv/Sum.idl
// generated code does not contain a copyright notice

#ifndef SUM_SRV_INTERFACE__SRV__DETAIL__SUM__TRAITS_HPP_
#define SUM_SRV_INTERFACE__SRV__DETAIL__SUM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sum_srv_interface/srv/detail/sum__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sum_srv_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Sum_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Sum_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Sum_Request & msg, bool use_flow_style = false)
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

}  // namespace sum_srv_interface

namespace rosidl_generator_traits
{

[[deprecated("use sum_srv_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sum_srv_interface::srv::Sum_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  sum_srv_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sum_srv_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const sum_srv_interface::srv::Sum_Request & msg)
{
  return sum_srv_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<sum_srv_interface::srv::Sum_Request>()
{
  return "sum_srv_interface::srv::Sum_Request";
}

template<>
inline const char * name<sum_srv_interface::srv::Sum_Request>()
{
  return "sum_srv_interface/srv/Sum_Request";
}

template<>
struct has_fixed_size<sum_srv_interface::srv::Sum_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sum_srv_interface::srv::Sum_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sum_srv_interface::srv::Sum_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace sum_srv_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Sum_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: sum
  {
    out << "sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sum, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Sum_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sum, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Sum_Response & msg, bool use_flow_style = false)
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

}  // namespace sum_srv_interface

namespace rosidl_generator_traits
{

[[deprecated("use sum_srv_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sum_srv_interface::srv::Sum_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  sum_srv_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sum_srv_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const sum_srv_interface::srv::Sum_Response & msg)
{
  return sum_srv_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<sum_srv_interface::srv::Sum_Response>()
{
  return "sum_srv_interface::srv::Sum_Response";
}

template<>
inline const char * name<sum_srv_interface::srv::Sum_Response>()
{
  return "sum_srv_interface/srv/Sum_Response";
}

template<>
struct has_fixed_size<sum_srv_interface::srv::Sum_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sum_srv_interface::srv::Sum_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sum_srv_interface::srv::Sum_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sum_srv_interface::srv::Sum>()
{
  return "sum_srv_interface::srv::Sum";
}

template<>
inline const char * name<sum_srv_interface::srv::Sum>()
{
  return "sum_srv_interface/srv/Sum";
}

template<>
struct has_fixed_size<sum_srv_interface::srv::Sum>
  : std::integral_constant<
    bool,
    has_fixed_size<sum_srv_interface::srv::Sum_Request>::value &&
    has_fixed_size<sum_srv_interface::srv::Sum_Response>::value
  >
{
};

template<>
struct has_bounded_size<sum_srv_interface::srv::Sum>
  : std::integral_constant<
    bool,
    has_bounded_size<sum_srv_interface::srv::Sum_Request>::value &&
    has_bounded_size<sum_srv_interface::srv::Sum_Response>::value
  >
{
};

template<>
struct is_service<sum_srv_interface::srv::Sum>
  : std::true_type
{
};

template<>
struct is_service_request<sum_srv_interface::srv::Sum_Request>
  : std::true_type
{
};

template<>
struct is_service_response<sum_srv_interface::srv::Sum_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SUM_SRV_INTERFACE__SRV__DETAIL__SUM__TRAITS_HPP_
