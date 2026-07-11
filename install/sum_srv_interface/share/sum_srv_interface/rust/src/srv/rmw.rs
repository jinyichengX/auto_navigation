#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "sum_srv_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sum_srv_interface__srv__Sum_Request() -> *const std::ffi::c_void;
}

#[link(name = "sum_srv_interface__rosidl_generator_c")]
extern "C" {
    fn sum_srv_interface__srv__Sum_Request__init(msg: *mut Sum_Request) -> bool;
    fn sum_srv_interface__srv__Sum_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Sum_Request>, size: usize) -> bool;
    fn sum_srv_interface__srv__Sum_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Sum_Request>);
    fn sum_srv_interface__srv__Sum_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Sum_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Sum_Request>) -> bool;
}

// Corresponds to sum_srv_interface__srv__Sum_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Sum_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub a: i64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub b: i64,

}



impl Default for Sum_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sum_srv_interface__srv__Sum_Request__init(&mut msg as *mut _) {
        panic!("Call to sum_srv_interface__srv__Sum_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Sum_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sum_srv_interface__srv__Sum_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sum_srv_interface__srv__Sum_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sum_srv_interface__srv__Sum_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Sum_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Sum_Request where Self: Sized {
  const TYPE_NAME: &'static str = "sum_srv_interface/srv/Sum_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sum_srv_interface__srv__Sum_Request() }
  }
}


#[link(name = "sum_srv_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sum_srv_interface__srv__Sum_Response() -> *const std::ffi::c_void;
}

#[link(name = "sum_srv_interface__rosidl_generator_c")]
extern "C" {
    fn sum_srv_interface__srv__Sum_Response__init(msg: *mut Sum_Response) -> bool;
    fn sum_srv_interface__srv__Sum_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Sum_Response>, size: usize) -> bool;
    fn sum_srv_interface__srv__Sum_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Sum_Response>);
    fn sum_srv_interface__srv__Sum_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Sum_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Sum_Response>) -> bool;
}

// Corresponds to sum_srv_interface__srv__Sum_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Sum_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub sum: i64,

}



impl Default for Sum_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sum_srv_interface__srv__Sum_Response__init(&mut msg as *mut _) {
        panic!("Call to sum_srv_interface__srv__Sum_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Sum_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sum_srv_interface__srv__Sum_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sum_srv_interface__srv__Sum_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sum_srv_interface__srv__Sum_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Sum_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Sum_Response where Self: Sized {
  const TYPE_NAME: &'static str = "sum_srv_interface/srv/Sum_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sum_srv_interface__srv__Sum_Response() }
  }
}






#[link(name = "sum_srv_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__sum_srv_interface__srv__Sum() -> *const std::ffi::c_void;
}

// Corresponds to sum_srv_interface__srv__Sum
#[allow(missing_docs, non_camel_case_types)]
pub struct Sum;

impl rosidl_runtime_rs::Service for Sum {
    type Request = Sum_Request;
    type Response = Sum_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__sum_srv_interface__srv__Sum() }
    }
}


