use nalgebra::{Matrix, Matrix4, Const, Vector3};

// Define custom types for fixed-size matrices and vectors

// nalgebra doesn't support all the sizes of vectors and matrices we need out of the box
// but we can create custom sizes like this
type Vector7 = Matrix<f64, Const<7>, Const<1>, nalgebra::ArrayStorage<f64, 7, 1>>;
type Matrix7 = Matrix<f64, Const<7>, Const<7>, nalgebra::ArrayStorage<f64, 7, 7>>;
type Matrix3 = Matrix<f64, Const<3>, Const<3>, nalgebra::ArrayStorage<f64, 3, 3>>;
type Matrix3x7 = Matrix<f64, Const<3>, Const<7>, nalgebra::ArrayStorage<f64, 3, 7>>;
type Vector4 = Matrix<f64, Const<4>, Const<1>, nalgebra::ArrayStorage<f64, 4, 1>>;

pub const GRAVITY: f64 = 9.81; // Gravitational constant (m/s^2)

pub struct EKF {
    pub state: Vector7,                 // State vector: [q0, q1, q2, q3, ωx, ωy, ωz]
    pub covariance: Matrix7,            // Covariance matrix P
    pub process_noise: Matrix7,         // Process noise Q
    pub measurement_noise: Matrix3,     // Measurement noise R
    pub dt: f64,                        // Time step (e.g., 0.01 for 100 Hz)
}

impl EKF {
    /// Create a new EKF instance
    pub fn new() -> Self {
        // Initialize Q (process noise covariance matrix)

        // List out all the indexes for filter tuning
        let mut process_noise = Matrix7::zeros();
        process_noise[(0, 0)] = 1e-3; // q0
        process_noise[(1, 1)] = 1e-3; // q1
        process_noise[(2, 2)] = 1e-3; // q2
        process_noise[(3, 3)] = 1e-3; // q3
        process_noise[(4, 4)] = 1e-1; // ωx
        process_noise[(5, 5)] = 1e-1; // ωy
        process_noise[(6, 6)] = 1e-1; // ωz

        // Initialize R (measurement noise covariance matrix)
        let mut measurement_noise = Matrix3::zeros();
        measurement_noise[(0, 0)] = 1e-4; // accel x
        measurement_noise[(1, 1)] = 1e-4; // accel y
        measurement_noise[(2, 2)] = 1e-4; // accel z

        EKF {
            state: {
                let mut state = Vector7::zeros();
                state[0] = 1.0; // Identity quaternion
                state
            },
            covariance: Matrix7::identity() * 1.0, // Initial state covariance
            process_noise,
            measurement_noise,
            dt: 0.005, // System frequency 200 Hz
        }
    }

    /// Predict step using gyro data
    pub fn predict(&mut self, gyro: [f64; 3]) {

        // Extract current quaternion
        let dt = self.dt;
        let omega = Vector3::new(gyro[0], gyro[1], gyro[2]);//0.0); // Ignore yaw (set ωz = 0)
    
        // Extract current quaternion
        let q = Vector4::new(self.state[0], self.state[1], self.state[2], self.state[3]);
    
        // Compute quaternion derivative: q_dot = 0.5 * Ω(q) * q
        let omega_matrix = Self::omega_matrix(omega);
        let q_dot = 0.5 * omega_matrix * q;
    
        // Integrate quaternion
        let q_new = q + q_dot * dt;
    
        // Normalize quaternion
        let norm = q_new.norm();
        if norm > 0.0 {
            let q_new = q_new / norm;
            self.state[0] = q_new[0];
            self.state[1] = q_new[1];
            self.state[2] = q_new[2];
            self.state[3] = q_new[3]; //0.0;
        }
    
        // Update angular velocity in state
        self.state[4] = gyro[0];
        self.state[5] = gyro[1];
        self.state[6] = gyro[2]; //0.0; // Ignore yaw rate (ωz)

        // Compute dynamic Jacobian (∂f/∂x)
        let f_jacobian = self.compute_f_jacobian(gyro);

        // Predict covariance: P' = FPFᵀ + Q
        self.covariance = f_jacobian * self.covariance * f_jacobian.transpose() + self.process_noise;
    }

    /// Update step using accelerometer data
    pub fn update(&mut self, accel: [f64; 3]) {
        // Extract quaternion from the state
        let mut q = Vector4::new(self.state[0], self.state[1], self.state[2], self.state[3]);
    
        // Compute expected accelerometer measurement: h(x) = R^T * g
        let gravity = Vector3::new(0.0, 0.0, -GRAVITY);
        let r_transpose = Self::quaternion_to_rotation_matrix(q).transpose();
        let accel_expected = r_transpose * gravity;
    
        // Compute the innovation: y = z - h(x)
        let z = Vector3::new(accel[0], accel[1], accel[2]); // Measured accelerometer data
        let innovation = z - accel_expected;
    
        // Compute measurement Jacobian (∂h/∂x)
        let h_jacobian = self.compute_h_jacobian(q);
    
        // Compute innovation covariance: S = HPHᵀ + R
        let s = h_jacobian * self.covariance * h_jacobian.transpose() + self.measurement_noise;
    
        // Compute Kalman gain: K = P Hᵀ S⁻¹
        let k = self.covariance * h_jacobian.transpose() * s.try_inverse().unwrap();
    
        // Update quaternion with roll and pitch corrections only
        //q[3] = 0.0; // Fix yaw (q3) component
        self.state += k * innovation;

        // Explicitly zero out yaw-related components in the quaternion
        //self.state[3] = 0.0; // Set yaw-related quaternion component to zero
        //self.state[6] = 0.0; // Set yaw rate to zero
    
        // Update covariance: P = (I - KH)P
        let i = Matrix7::identity();
        self.covariance = (i - k * h_jacobian) * self.covariance;
    
        // Normalize quaternion after correction
        let norm = q.norm();
        if norm > 0.0 {
            q = q / norm;
        }
    
        self.state[0] = q[0] / norm;
        self.state[1] = q[1] / norm;
        self.state[2] = q[2] / norm;
        self.state[3] = q[3] / norm;
        
    }
    
    /// Compute the dynamic Jacobian (∂f/∂x)
    fn compute_f_jacobian(&self, gyro: [f64; 3]) -> Matrix7 {
        let p = gyro[0];
        let q = gyro[1];
        let r = gyro[2];
        let dt = self.dt;

        let mut f = Matrix7::identity();
        f[(0, 1)] = -p * dt;
        f[(0, 2)] = -q * dt;
        f[(0, 3)] = -r * dt;
        f[(1, 0)] = p * dt;
        f[(1, 2)] = r * dt;
        f[(1, 3)] = -q * dt;
        f[(2, 0)] = q * dt;
        f[(2, 1)] = -r * dt;
        f[(2, 3)] = p * dt;
        f[(3, 0)] = r * dt;
        f[(3, 1)] = q * dt;
        f[(3, 2)] = -p * dt;
        f
    }

    /// Compute the measurement Jacobian (∂h/∂x)
    fn compute_h_jacobian(&self, q: Vector4) -> Matrix3x7 {
        let q0 = q[0];
        let q1 = q[1];
        let q2 = q[2];
        let q3 = q[3];

        let mut h = Matrix3x7::zeros();
        h[(0, 0)] = 2.0 * (-GRAVITY * q2);
        h[(0, 1)] = 2.0 * (GRAVITY * q3);
        h[(0, 2)] = 2.0 * (-GRAVITY * q0);
        h[(0, 3)] = 2.0 * (GRAVITY * q1);
        h[(1, 0)] = 2.0 * (GRAVITY * q1);
        h[(1, 1)] = 2.0 * (GRAVITY * q0);
        h[(1, 2)] = 2.0 * (GRAVITY * q3);
        h[(1, 3)] = 2.0 * (GRAVITY * q2);
        h[(2, 0)] = 2.0 * (GRAVITY * q0);
        h[(2, 1)] = 2.0 * (-GRAVITY * q1);
        h[(2, 2)] = 2.0 * (-GRAVITY * q2);
        h[(2, 3)] = 2.0 * (-GRAVITY * q3);
        h
    }

    /// Convert quaternion to rotation matrix
    fn quaternion_to_rotation_matrix(q: Vector4) -> Matrix3 {
        let q0 = q[0];
        let q1 = q[1];
        let q2 = q[2];
        let q3 = q[3];

        Matrix3::new(
            1.0 - 2.0 * (q2 * q2 + q3 * q3),
            2.0 * (q1 * q2 - q0 * q3),
            2.0 * (q1 * q3 + q0 * q2),
            2.0 * (q1 * q2 + q0 * q3),
            1.0 - 2.0 * (q1 * q1 + q3 * q3),
            2.0 * (q2 * q3 - q0 * q1),
            2.0 * (q1 * q3 - q0 * q2),
            2.0 * (q2 * q3 + q0 * q1),
            1.0 - 2.0 * (q1 * q1 + q2 * q2),
        )
    }

    /// Compute omega matrix for quaternion dynamics
    fn omega_matrix(omega: Vector3<f64>) -> Matrix<f64, Const<4>, Const<4>, nalgebra::ArrayStorage<f64, 4, 4>> {
        Matrix4::new(
            0.0, -omega[0], -omega[1], -omega[2],
            omega[0], 0.0, omega[2], -omega[1],
            omega[1], -omega[2], 0.0, omega[0],
            omega[2], omega[1], -omega[0], 0.0,
        )
    }


    /// Get the fully updated state vector
    pub fn get_state(&self) -> Vector7 {
        self.state.clone() // Return a copy of the state vector
    }
    
    
}
