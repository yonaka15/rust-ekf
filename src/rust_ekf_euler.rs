use nalgebra::{Matrix3, Matrix3x6, Matrix6, Vector6, Vector3};

pub const GRAVITY: f64 = 9.81; // Gravitational constant (m/s^2)

// EKFEuler Struct
pub struct EKFEuler {
    pub state: Vector6<f64>,                // State Vector: [roll (phi), pitch (theta), yaw (psi), roll rate (p), pitch rate (q), yaw rate (r)]
    pub covariance: Matrix6<f64>,           // Covariance matrix P
    pub process_noise: Matrix6<f64>,        // Process noise Q
    pub measurement_noise: Matrix3<f64>,    // Measurement noise R
    pub dt: f64,                            // Time step (0.01 for 100 Hz)
    pub airspeed: f64,                      // Airspeed (v_a), initialized at 0
}

impl EKFEuler {
    // Create a new EKFEuler instance
    pub fn new() -> Self {
        EKFEuler {
            state: Vector6::zeros(),                        // Initial state: zero roll, pitch, yaw, and angular rates
            covariance: Matrix6::identity() * 1.0,          // Initialize P with some uncertainty
            process_noise: Matrix6::identity() * 0.1,       // Process noise Q (TUNED EXPERIMENTALLY)
            measurement_noise: Matrix3::identity() * 0.2,    // Measurement noise R (TUNED EXPERIMENTALLY)
            dt: 0.05,
            airspeed: 0.0,                                  // Assume airspeed is 0 for now; future nonzero airspeed compatibility included
        }
    }

    pub fn predict(&mut self, gyro: [f64; 3]) {
        //Extract state variables for readability
        let phi = self.state[0];                // Roll angle
        let theta = self.state[1];              // Pitch angle
        let _psi = self.state[2];                // Yaw angle
        let p = gyro[0];                        // Roll rate (gyro x)
        let q = gyro[1];                        // Pitch rate (gyro y)
        let r = gyro[2];                        // Yaw rate (gyro z)
        let dt = self.dt;                       // dt

        // Dynamics: f(x, u)
        let roll_dot = p + q * phi.sin() * theta.tan() + r * phi.cos() * theta.tan();
        let pitch_dot = q * phi.cos() - r * phi.sin();
        let yaw_dot = r; // Yaw is simple integration of yaw rate

        // Update state with predicted dynamics
        self.state[0] += roll_dot * dt;         // Update roll
        self.state[1] += pitch_dot * dt;        // Update pitch
        self.state[2] += yaw_dot * dt;          // Update yaw
        self.state[3] = p;                      // Update roll rate
        self.state[4] = q;                      // Update pitch rate
        self.state[5] = r;                      // Update yaw rate

        // Jacobian of dynamics: ∂f/∂x
        let mut f_jacobian = Matrix6::identity();
        // Roll dynamics (first row)
        f_jacobian[(0, 0)] = (q * phi.cos() * theta.tan() - r * phi.sin() * theta.tan()) * dt;    // ∂roll_dot/∂phi
        f_jacobian[(0, 1)] = ((q * phi.sin() - r * phi.cos()) / theta.cos().powi(2)) * dt;        // ∂roll_dot/∂theta
        // Pitch dynamics (second row)
        f_jacobian[(1, 0)] = (-q * phi.sin() - r * phi.cos()) * dt; // ∂pitch_dot/∂phi
        f_jacobian[(1, 1)] = 0.0; // No significant dependency of ∂pitch_dot/∂theta
        // Yaw dynamics (third row)
        f_jacobian[(2, 2)] = 0.0; // ∂yaw_dot/∂yaw
        f_jacobian[(2, 5)] = 1.0; // ∂yaw_dot/∂r
        // Angular rates (rows 4, 5, 6) remain identity
        // These entries are unaffected by dynamics and stay initialized to 1.0


        // Predict covariance: P' = FPFᵀ + Q
        self.covariance = self.covariance + (dt * (f_jacobian * self.covariance * f_jacobian.transpose() + self.process_noise));
    }

    /// Update step (nonlinear measurement model)
    pub fn update(&mut self, accel: [f64;3]) {
        // Extract state variables for readability
        let phi = self.state[0]; // Roll angle
        let theta = self.state[1]; // Pitch angle
        let p = self.state[3];                        // Roll rate (gyro x)
        let q = self.state[4];                        // Pitch rate (gyro y)
        let r = self.state[5];                        // Yaw rate (gyro z)
        let v_a = self.airspeed;                        // Airspeed 9assumed 0 for now)


        // Measurement Model: h(x)
        let h_roll = v_a * q * theta.sin() + GRAVITY * theta.sin(); // q V_a sin(theta) + g sin(theta)
        let h_pitch = v_a * (r * theta.cos() - p * theta.sin()) - GRAVITY * theta.cos() * phi.sin(); // r V_a cos(theta) - p V_a sin(theta) - g cos(theta) sin(phi)
        let h_yaw = -v_a * q * theta.cos() - GRAVITY * theta.cos() * phi.cos(); // -q V_a cos(theta) - g cos(theta) cos(phi)
        let h = Vector3::new(h_roll, h_pitch, h_yaw);

        // Innovation: y = z - h(x)
        let z = Vector3::new(accel[0], accel[1], 0.0); // Measured accelerometer data; z left as 0
        let y = z - h;

        // Jacobian of measurement model: ∂h/∂x
        let mut h_jacobian = Matrix3x6::zeros();

        // Row 1: h_roll (qV_a sin θ + g sin θ)
        h_jacobian[(0, 1)] = v_a * q * theta.cos() + GRAVITY * theta.cos(); // ∂h_roll/∂theta

        // Row 2: h_pitch (rV_a cos θ - pV_a cos θ + g sin φ sin θ)
        h_jacobian[(1, 0)] = -GRAVITY * phi.cos() * theta.cos(); // ∂h_pitch/∂phi
        h_jacobian[(1, 1)] = -r * v_a * theta.sin() - p * v_a * theta.cos() + GRAVITY * phi.sin() * theta.sin(); // ∂h_pitch/∂theta

        // Row 3: h_yaw (-qV_a cos θ - g cos θ cos φ)
        h_jacobian[(2, 0)] = GRAVITY * phi.sin() * theta.cos(); // ∂h_yaw/∂phi
        h_jacobian[(2, 1)] = (q * v_a + GRAVITY * phi.cos()) * theta.sin(); // ∂h_yaw/∂thetas

        // Innovation covariance: S = HPHᵀ + R
        let s = h_jacobian * self.covariance * h_jacobian.transpose() + self.measurement_noise;

        // Kalman gain: K = P Hᵀ S⁻¹
        let k = self.covariance * h_jacobian.transpose() * s.try_inverse().unwrap();

        // Update state: x = x + Ky
        self.state = self.state + k * y;

        // Update covariance: P = (I - KH)P
        let i = Matrix6::identity();
        self.covariance = (i - k * h_jacobian) * self.covariance;

        // NOTE: Yaw is not updated here because we lack a sensor (e.g., magnetometer or GPS)
        // Future improvements could add yaw correction using those sensors

        }

    /// Get the current state estimate
    pub fn get_state(&self) -> Vector6<f64> {
        self.state
    }

}