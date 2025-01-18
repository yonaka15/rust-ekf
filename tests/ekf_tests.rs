use rust_ekf::{EKF, GRAVITY}; // Import EKF struct and GRAVITY constant

#[test]
fn test_initialization() {
    // Test that the EKF struct initializes correctly
    let ekf = EKF::new();
    assert_eq!(ekf.state.len(), 7); // State vector should have 7 elements
    assert_eq!(ekf.state[0], 1.0); // Quaternion should start as identity
    assert_eq!(ekf.state[1], 0.0);
    assert_eq!(ekf.state[2], 0.0);
    assert_eq!(ekf.state[3], 0.0);
    assert!(ekf.covariance.is_square()); // Covariance matrix should be square
    assert!(ekf.process_noise.is_square()); // Process noise matrix should be square
    assert!(ekf.measurement_noise.is_square()); // Measurement noise matrix should be square
}

#[test]
fn test_predict() {
    // Test that the predict method works correctly with sample gyro data
    let mut ekf = EKF::new();
    let gyro_data = [0.1, 0.2, 0.3]; // Sample gyro data (rad/s)
    
    ekf.predict(gyro_data);
    
    // Assert that the quaternion state has been updated
    assert_ne!(ekf.state[0], 1.0); // Quaternion w-component should change
    assert_ne!(ekf.state[1], 0.0); // Quaternion x-component
    assert_ne!(ekf.state[2], 0.0); // Quaternion y-component
    assert_ne!(ekf.state[3], 0.0); // Quaternion z-component

    // Assert that angular velocity is updated
    assert_eq!(ekf.state[4], gyro_data[0]); // ωx
    assert_eq!(ekf.state[5], gyro_data[1]); // ωy
    assert_eq!(ekf.state[6], gyro_data[2]); // ωz
}

#[test]
fn test_update() {
    // Test that the update method works correctly with sample accel and gyro data
    let mut ekf = EKF::new();
    let accel_data = [0.0, 0.0, GRAVITY]; // Sample accelerometer data (m/s^2)
    let gyro_data = [0.1, 0.2, 0.3]; // Sample gyro data (rad/s)

    ekf.predict(gyro_data); // Run predict phase
    ekf.update(accel_data); // Run update phase

    // Assert that the state vector is updated correctly
    assert_ne!(ekf.state[0], 1.0); // Quaternion w-component
    assert_ne!(ekf.state[1], 0.0); // Quaternion x-component
    assert_ne!(ekf.state[2], 0.0); // Quaternion y-component
    assert_ne!(ekf.state[3], 0.0); // Quaternion z-component

    // Assert that the quaternion remains normalized after update
    let quaternion_norm = (ekf.state[0].powi(2)
        + ekf.state[1].powi(2)
        + ekf.state[2].powi(2)
        + ekf.state[3].powi(2))
    .sqrt();
    assert!((quaternion_norm - 1.0).abs() < 1e-6);
}

#[test]
fn test_get_state() {
    // Test that the get_state method returns the correct state
    let mut ekf = EKF::new();
    let gyro_data = [0.1, 0.2, 0.3]; // Sample gyro data (rad/s)
    ekf.predict(gyro_data);

    let state = ekf.get_state();
    assert_eq!(state.len(), 7); // State vector should have 7 elements

    // Assert that the quaternion remains normalized
    let quaternion_norm = (state[0].powi(2)
        + state[1].powi(2)
        + state[2].powi(2)
        + state[3].powi(2))
    .sqrt();
    assert!((quaternion_norm - 1.0).abs() < 1e-6);

    // Check that angular velocities match the latest gyro input
    assert_eq!(state[4], 0.1); // ωx
    assert_eq!(state[5], 0.2); // ωy
    assert_eq!(state[6], 0.3); // ωz
}
