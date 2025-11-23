// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/GetVelocity.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__GET_VELOCITY__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__GET_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/get_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetVelocity_Request_id
{
public:
  Init_GetVelocity_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::GetVelocity_Request id(::custom_interfaces::srv::GetVelocity_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::GetVelocity_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::GetVelocity_Request>()
{
  return custom_interfaces::srv::builder::Init_GetVelocity_Request_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetVelocity_Response_velocity
{
public:
  Init_GetVelocity_Response_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::GetVelocity_Response velocity(::custom_interfaces::srv::GetVelocity_Response::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::GetVelocity_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::GetVelocity_Response>()
{
  return custom_interfaces::srv::builder::Init_GetVelocity_Response_velocity();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__GET_VELOCITY__BUILDER_HPP_
