#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to status_interface__msg__SystemStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SystemStatus {
    /// 记录时间戳
    pub stamp: builtin_interfaces::msg::Time,

    /// 系统名称
    pub host_name: std::string::String,

    /// CPU使用率
    pub cpu_percent: f32,

    /// 内存使用率
    pub memory_percent: f32,

    /// 内存总量
    pub memory_total: f32,

    /// 剩余有效内存
    pub memory_available: f32,

    /// 网络发送数据总量
    pub net_sent: f64,

    /// 网络接收数据总量
    pub net_recv: f64,

}



impl Default for SystemStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SystemStatus::default())
  }
}

impl rosidl_runtime_rs::Message for SystemStatus {
  type RmwMsg = super::msg::rmw::SystemStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        host_name: msg.host_name.as_str().into(),
        cpu_percent: msg.cpu_percent,
        memory_percent: msg.memory_percent,
        memory_total: msg.memory_total,
        memory_available: msg.memory_available,
        net_sent: msg.net_sent,
        net_recv: msg.net_recv,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        host_name: msg.host_name.as_str().into(),
      cpu_percent: msg.cpu_percent,
      memory_percent: msg.memory_percent,
      memory_total: msg.memory_total,
      memory_available: msg.memory_available,
      net_sent: msg.net_sent,
      net_recv: msg.net_recv,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      host_name: msg.host_name.to_string(),
      cpu_percent: msg.cpu_percent,
      memory_percent: msg.memory_percent,
      memory_total: msg.memory_total,
      memory_available: msg.memory_available,
      net_sent: msg.net_sent,
      net_recv: msg.net_recv,
    }
  }
}


