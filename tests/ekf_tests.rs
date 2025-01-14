use ekf::{EKF, GRAVITY}; // Import EKF struct and GRAVITY constant

#[test]
fn test_initialization() {
    // Test that the EKF struct initializes correctly
    let ekf = EKF::new();
    assert_eq!(ekf.state.len(), 6); // State vector should have 6 elements
    assert!(ekf.covariance.is_square()); // Covariance matrix should be square
    assert!(ekf.process_noise.is_square()); // Process noise matrix should be square
    assert!(ekf.measurement_noise.is_square()); // Measurement noise matrix should be square
    assert_eq!(ekf.airspeed, 0.0); // Airspeed should initialize to 0
}

#[test]
fn test_predict() {
    // Test that the predict method works correctly with sample gyro data
    let mut ekf = EKF::new();
    let gyro_data = [0.1, 0.2, 0.3]; // Sample gyro data (rad/s)
    
    ekf.predict(gyro_data);
    
    // Assert that the state has been updated (values will depend on the equations)
    assert_ne!(ekf.state[0], 0.0); // Roll (phi)
    assert_ne!(ekf.state[1], 0.0); // Pitch (theta)
    assert_ne!(ekf.state[2], 0.0); // Yaw (psi)
}

#[test]
fn test_update() {
    // Test that the update method works correctly with sample accel and gyro data
    let mut ekf = EKF::new();
    let accel_data = [0.0, 0.0, GRAVITY]; // Sample accelerometer data (m/s^2)
    let gyro_data = [0.1, 0.2, 0.3]; // Sample gyro data (rad/s)

    ekf.predict(gyro_data);
    ekf.update(accel_data);

    // Assert that the state has been updated (values will depend on the equations)
    assert_ne!(ekf.state[0], 0.0); // Roll (phi)
    assert_ne!(ekf.state[1], 0.0); // Pitch (theta)
    assert_ne!(ekf.state[2], 0.0); // Yaw (psi)
}

#[test]
fn test_get_state() {
    // Test that the get_state method returns the correct state
    let mut ekf = EKF::new();
    let gyro_data = [0.1, 0.2, 0.3]; // Sample gyro data (rad/s)
    ekf.predict(gyro_data);

    let state = ekf.get_state();
    assert_eq!(state.len(), 6); // State vector should have 6 elements
    assert_eq!(state[3], 0.1); // Roll rate (p)
    assert_eq!(state[4], 0.2); // Pitch rate (q)
    assert_eq!(state[5], 0.3); // Yaw rate (r)
}
