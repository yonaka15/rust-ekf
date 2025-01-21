use nalgebra::{Matrix, Matrix4, Const, Vector3};

// Define custom types for fixed-size matrices and vectors

// nalgebra doesn't support all the sizes of vectors and matrices that we need out of the box
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
}

impl EKF {
    /// Create a new EKF instance, passing accelerometer data to calculate the initial quaternion (avoids using 0's for initial orientation)
    pub fn new(accel_data: Option<[f64; 3]>) -> Self {
        let (q0, q1, q2, q3) = if let Some(accel_data) = accel_data {
            // Normalize accelerometer vector
            let norm = (accel_data[0].powi(2) + accel_data[1].powi(2) + accel_data[2].powi(2)).sqrt();
            let ax = accel_data[0] / norm;
            let ay = accel_data[1] / norm;
            let az = -accel_data[2] / norm;
    
            // Calculate quaternion from accelerometer data
            let q0 = (1.0 + az).sqrt() / 2.0;
            let q1 = -ay / (2.0 * q0);
            let q2 = ax / (2.0 * q0);
            let q3: f64 = 0.0; // Yaw is zero since accelerometer data cannot calulcate yaw angles

            let norm = (q0.powi(2) + q1.powi(2) + q2.powi(2) + q3.powi(2)).sqrt();
            let q0 = q0 / norm;
            let q1 = q1 / norm;
            let q2 = q2 / norm;
            let q3 = q3 / norm;
            

            (q0, q1, q2, q3)
        } else {
            // Default to identity quaternion
            (1.0, 0.0, 0.0, 0.0)
        };
    
        // Initialize process and measurement noise matrices
        let mut process_noise = Matrix7::zeros();
        process_noise[(0, 0)] = 1e-5; // q0
        process_noise[(1, 1)] = 1e-5; // q1
        process_noise[(2, 2)] = 1e-5; // q2
        process_noise[(3, 3)] = 1e-5; // q3
        process_noise[(4, 4)] = 1e-5; // ωx
        process_noise[(5, 5)] = 1e-5; // ωy
        process_noise[(6, 6)] = 1e-5; // ωz
        
        let mut measurement_noise = Matrix3::zeros();
        measurement_noise[(0, 0)] = 1e-1; // accel x
        measurement_noise[(1, 1)] = 1e-1; // accel y
        measurement_noise[(2, 2)] = 1e-1; // accel z
        
    
        EKF {
            state: {
                let mut state = Vector7::zeros();
                state[0] = q0;
                state[1] = q1;
                state[2] = q2;
                state[3] = q3;
                state[4] = 0.0; // ωx
                state[5] = 0.0; // ωy
                state[6] = 0.0; // ωz
                state
            },
            covariance: Matrix7::identity() * 1.0, // Initial state covariance
            process_noise,
            measurement_noise,
        }
    }
    
    /// Predict step using gyro data, pass real dt (computed dynamically)
    pub fn predict(&mut self, gyro: [f64; 3], dt: f64) {
        let omega = Vector3::new(gyro[0], gyro[1], gyro[2]); // gyro data vector
    
        // Extract current quaternion
        let q = Vector4::new(self.state[0], self.state[1], self.state[2], self.state[3]);
    
        // Compute quaternion derivative: q_dot = 0.5 * Ω(q) * q
        let omega_matrix = Self::omega_matrix(omega);
        let q_dot = 0.5 * omega_matrix * q;
        
        // Integrate quaternion
        let q_new = q + q_dot * dt;
    
        // Normalize quaternion to ensure quaternion remains unit quaternion
        // and set the first 4 indexes (quaternion values) to the integrated and normalized
        // values, q_new
        let norm = q_new.norm();
        if norm > 0.0 {
            let q_new = q_new / norm;
            self.state[0] = q_new[0];
            self.state[1] = q_new[1];
            self.state[2] = q_new[2];
            self.state[3] = q_new[3];
        }
    
        // Update angular velocity in state vector with the gyro measurements
        self.state[4] = gyro[0];
        self.state[5] = gyro[1];
        self.state[6] = gyro[2]; 

        // Compute dynamic Jacobian (∂f/∂x) using custom compute_f_jacobian method
        let f_jacobian = self.compute_f_jacobian(gyro, dt);

        // Predict covariance using: P' = FPFᵀ + Q
        self.covariance = f_jacobian * self.covariance * f_jacobian.transpose() + self.process_noise;
    }

    /// Update step using accelerometer data
    pub fn update(&mut self, accel: [f64; 3]) {
        // Extract quaternion from the state that was estimated with dynamics + gyro data in the predict phase
        let mut q = Vector4::new(self.state[0], self.state[1], self.state[2], self.state[3]);
    
        // Compute expected accelerometer measurement using the rotation matrix (calculated from quaternion) 
        // and gravitational matrix: h(x) = R^T * g
        let gravity = Vector3::new(0.0, 0.0, -GRAVITY);
        let r_transpose = Self::quaternion_to_rotation_matrix(q).transpose();
        let accel_expected = r_transpose * gravity;
    
        // Compute the innovation: y = z - h(x)
        let z = Vector3::new(accel[0], accel[1], accel[2]); // Measured accelerometer data
        let innovation = z - accel_expected;
    
        // Compute measurement Jacobian (∂h/∂x) using the custom compute_h_jacobian method
        let h_jacobian = self.compute_h_jacobian(q);
    
        // Compute innovation covariance: S = HPHᵀ + R
        let s = h_jacobian * self.covariance * h_jacobian.transpose() + self.measurement_noise;
    
        // Compute Kalman gain: K = P Hᵀ S⁻¹
        let k = self.covariance * h_jacobian.transpose() * s.try_inverse().unwrap();
    
        // Update state vector with new quaterion
        self.state += k * innovation;

        // Update covariance: P = (I - KH)P
        let i = Matrix7::identity();
        self.covariance = (i - k * h_jacobian) * self.covariance;
    
        // Normalize quaternion after correction (always normalize the quaternion after predict and update phase
        // to avoid accumulating error and ensure quaternion remains unit value)
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
    fn compute_f_jacobian(&self, gyro: [f64; 3], dt: f64) -> Matrix7 {
        let p = gyro[0];
        let q = gyro[1];
        let r = gyro[2];
        //let dt = self.dt;

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
            1.0 - 1.0 * (q2 * q2 + q3 * q3),
            2.0 * (q1 * q2 - q0 * q3),
            2.0 * (q1 * q3 + q0 * q2),
            2.0 * (q1 * q2 + q0 * q3),
            1.0 - 1.0 * (q1 * q1 + q3 * q3),
            2.0 * (q2 * q3 - q0 * q1),
            2.0 * (q1 * q3 - q0 * q2),
            2.0 * (q2 * q3 + q0 * q1),
            1.0 - 1.0 * (q1 * q1 + q2 * q2),
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
