#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "chapt4_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__chapt4_interfaces__srv__FaceDetector_Request() -> *const std::ffi::c_void;
}

#[link(name = "chapt4_interfaces__rosidl_generator_c")]
extern "C" {
    fn chapt4_interfaces__srv__FaceDetector_Request__init(msg: *mut FaceDetector_Request) -> bool;
    fn chapt4_interfaces__srv__FaceDetector_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FaceDetector_Request>, size: usize) -> bool;
    fn chapt4_interfaces__srv__FaceDetector_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FaceDetector_Request>);
    fn chapt4_interfaces__srv__FaceDetector_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FaceDetector_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FaceDetector_Request>) -> bool;
}

// Corresponds to chapt4_interfaces__srv__FaceDetector_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FaceDetector_Request {
    /// 人脸图像
    pub image: sensor_msgs::msg::rmw::Image,

}



impl Default for FaceDetector_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !chapt4_interfaces__srv__FaceDetector_Request__init(&mut msg as *mut _) {
        panic!("Call to chapt4_interfaces__srv__FaceDetector_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FaceDetector_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { chapt4_interfaces__srv__FaceDetector_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { chapt4_interfaces__srv__FaceDetector_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { chapt4_interfaces__srv__FaceDetector_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FaceDetector_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FaceDetector_Request where Self: Sized {
  const TYPE_NAME: &'static str = "chapt4_interfaces/srv/FaceDetector_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__chapt4_interfaces__srv__FaceDetector_Request() }
  }
}


#[link(name = "chapt4_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__chapt4_interfaces__srv__FaceDetector_Response() -> *const std::ffi::c_void;
}

#[link(name = "chapt4_interfaces__rosidl_generator_c")]
extern "C" {
    fn chapt4_interfaces__srv__FaceDetector_Response__init(msg: *mut FaceDetector_Response) -> bool;
    fn chapt4_interfaces__srv__FaceDetector_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FaceDetector_Response>, size: usize) -> bool;
    fn chapt4_interfaces__srv__FaceDetector_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FaceDetector_Response>);
    fn chapt4_interfaces__srv__FaceDetector_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FaceDetector_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FaceDetector_Response>) -> bool;
}

// Corresponds to chapt4_interfaces__srv__FaceDetector_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FaceDetector_Response {
    /// 人脸个数
    pub number: i16,

    /// 识别耗时
    pub use_time: f32,

    /// 人脸在图像中的位置
    pub top: rosidl_runtime_rs::Sequence<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right: rosidl_runtime_rs::Sequence<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bottom: rosidl_runtime_rs::Sequence<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left: rosidl_runtime_rs::Sequence<i32>,

}



impl Default for FaceDetector_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !chapt4_interfaces__srv__FaceDetector_Response__init(&mut msg as *mut _) {
        panic!("Call to chapt4_interfaces__srv__FaceDetector_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FaceDetector_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { chapt4_interfaces__srv__FaceDetector_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { chapt4_interfaces__srv__FaceDetector_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { chapt4_interfaces__srv__FaceDetector_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FaceDetector_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FaceDetector_Response where Self: Sized {
  const TYPE_NAME: &'static str = "chapt4_interfaces/srv/FaceDetector_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__chapt4_interfaces__srv__FaceDetector_Response() }
  }
}






#[link(name = "chapt4_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__chapt4_interfaces__srv__FaceDetector() -> *const std::ffi::c_void;
}

// Corresponds to chapt4_interfaces__srv__FaceDetector
#[allow(missing_docs, non_camel_case_types)]
pub struct FaceDetector;

impl rosidl_runtime_rs::Service for FaceDetector {
    type Request = FaceDetector_Request;
    type Response = FaceDetector_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__chapt4_interfaces__srv__FaceDetector() }
    }
}


