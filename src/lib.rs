pub mod rust_ekf_euler;
pub mod rust_ekf_quaternion;

pub use rust_ekf_euler::{EKFEuler, GRAVITY as GRAVITY_EULER};
pub use rust_ekf_quaternion::{EKF, GRAVITY};