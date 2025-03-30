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
        process_noise[(0, 0)] = 1e-7; // q0
        process_noise[(1, 1)] = 1e-7; // q1
        process_noise[(2, 2)] = 1e-7; // q2
        process_noise[(3, 3)] = 1e-7; // q3
        process_noise[(4, 4)] = 1e-3; // ωx
        process_noise[(5, 5)] = 1e-3; // ωy
        process_noise[(6, 6)] = 1e-3; // ωz
        
        let mut measurement_noise = Matrix3::zeros();
        measurement_noise[(0, 0)] = 5e-2; // accel x
        measurement_noise[(1, 1)] = 5e-2; // accel y
        measurement_noise[(2, 2)] = 5e-2; // accel z 

    
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
    
    /// Predict step using gyro data. Pass real dt (computed dynamically)
    pub fn predict(&mut self, gyro: [f64; 3], dt: f64) {
        // 1. Convert gyro readings into a vector (angular velocities in rad/s)
        let omega = Vector3::new(gyro[0], gyro[1], gyro[2]);

        // 2. Extract current quaternion from the state vector
        let q = Vector4::new(self.state[0], self.state[1], self.state[2], self.state[3]);

        // 3. Compute quaternion derivative: q̇ = 0.5 * Ω(ω) * q
        let omega_matrix = Self::omega_matrix(omega);
        let q_dot = 0.5 * omega_matrix * q;

        // 4. Integrate quaternion using Euler integration: q_new = q + q̇ * dt
        let q_new = q + q_dot * dt;

        // 5. Update the quaternion in the state vector (first 4 elements)
        self.state.fixed_rows_mut::<4>(0).copy_from(&q_new);

        // 6. Normalize quaternion to maintain unit length (numerical stability)
        Self::normalize_quaternion_in_state(&mut self.state);

        // 7. Update angular rates in the state vector with the latest gyro readings
        self.state[4] = gyro[0]; // ωx
        self.state[5] = gyro[1]; // ωy
        self.state[6] = gyro[2]; // ωz

        // 8. Compute dynamics Jacobian ∂f/∂x based on current gyro readings and dt
        let f_jacobian = self.compute_f_jacobian(gyro, dt);

        // 9. Propagate uncertainty using the covariance update: P' = FPFᵀ + Q
        self.covariance = f_jacobian * self.covariance * f_jacobian.transpose() + self.process_noise;
    }


    /// Update step using accelerometer data
    pub fn update(&mut self, accel: [f64; 3]) {
        // 1. Predict expected measurement (accel_expected) from quaternion
        let gravity = Vector3::new(0.0, 0.0, -GRAVITY);
        let q = Vector4::new(self.state[0], self.state[1], self.state[2], self.state[3]);
        let r_transpose = Self::quaternion_to_rotation_matrix(q).transpose();
        let accel_expected = r_transpose * gravity;

        // 2. Innovation: difference between real sensor data and predicted sensor output
        let z = Vector3::new(accel[0], accel[1], accel[2]);
        let innovation = z - accel_expected;

        // 3. Compute Measurement Jacobian
        let h_jacobian = self.compute_h_jacobian(q);

        // 4. Compute Innovation Covariance
        let s = h_jacobian * self.covariance * h_jacobian.transpose() + self.measurement_noise;

        // 5. Try to invert S
        let s_inv = match s.try_inverse() {
            Some(inv) => inv,
            None => {
                // Matrix not invertible — skip update to maintain stability
                eprintln!("Warning: Innovation covariance matrix is non-invertible. Skipping EKF update step.");
                return;
            }
        };

        // 6. Compute Kalman Gain
        let k = self.covariance * h_jacobian.transpose() * s_inv;

        // 7. Update state estimate
        self.state += k * innovation;

        // 8. Update covariance estimate
        let i = Matrix7::identity();
        self.covariance = (i - k * h_jacobian) * self.covariance;

        // 9. Normalize quaternion
        Self::normalize_quaternion_in_state(&mut self.state);
    }



    /// Compute the dynamic Jacobian (∂f/∂x)
    fn compute_f_jacobian(&self, gyro: [f64; 3], dt: f64) -> Matrix7 {
        let p = gyro[0];
        let q = gyro[1];
        let r = gyro[2];

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

    fn normalize_quaternion_in_state(state: &mut Vector7) {
        let q = Vector4::new(state[0], state[1], state[2], state[3]);
        let norm = q.norm();
        if norm > 0.0 {
            let q_normalized = q / norm;
            state.fixed_rows_mut::<4>(0).copy_from(&q_normalized);
        }
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
