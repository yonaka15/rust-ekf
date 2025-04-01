For a deep dive into this Rust Extended Kalman Filter, read my Medium article here.s

## How to Use `rust-ekf`

The `rust-ekf` library can be used in your Rust projects to implement an Extended Kalman Filter for state estimation. Follow the steps below to integrate it into your project.

### Adding `rust-ekf` as a Dependency

You can include `rust-ekf` as a dependency in your `Cargo.toml` by referencing the GitHub repository. Add the following lines to your `Cargo.toml`:

	[dependencies]
	rust-ekf = { git = "https://github.com/OrlandoQuintana/rust-ekf" }
This tells Cargo to pull the library directly from the GitHub repository and use it in your project.

----------

### Importing `rust-ekf` into Your Code

To use the library in your Rust code, import the `EKF` struct at the top of your file. 

Example:

	use rust_ekf::EKF;

This makes the `EKF` struct available for use in your code.

----------

### Using the `rust-ekf` Code Locally

If you prefer to clone the `rust-ekf` repository and use it as a local dependency, follow these steps:

1.  Clone the repository to your local machine:

		git clone https://github.com/OrlandoQuintana/rust-ekf.git
2. Place the cloned repository in a desired location on your computer.
3. Add the local path to your `Cargo.toml` dependencies:

		[dependencies]
		rust-ekf = { path = "../path/to/rust-ekf" }
	Replace `../path/to/rust-ekf` with the actual relative path to the `rust-ekf` folder.

### Example Usage in Your Code

Here’s an example of how you might use the `rust-ekf` library:
		
	use rust_ekf::EKF;

	fn main() {
	    // Create a new EKF instance
	    let mut ekf = EKF::new();

	    // Example gyroscope data (roll rate, pitch rate, yaw rate in rad/s)
	    let gyro_data = [0.01, -0.02, 0.03];
        let dt = 0.005; 

	    // Prediction phase
	    ekf.predict(gyro_data, dt);

	    // Example accelerometer data (x, y, z acceleration in m/s^2)
	    let accel_data = [0.0, 9.81, 0.0];

	    // Update phase
	    ekf.update(accel_data);

	    // Get the updated state vector
	    let state = ekf.get_state();
	    println!("Updated State Vector: {:?}", state);
	}