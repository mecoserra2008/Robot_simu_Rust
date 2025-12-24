use nalgebra as na;
use rand_distr::{Distribution, Normal};

/// IMU sensor (gyroscope + accelerometer)
#[derive(Debug, Clone)]
pub struct IMU {
    pub gyro_noise_std: f32,
    pub accel_noise_std: f32,
    pub sampling_rate: f32,
    pub gyro_bias: na::Vector3<f32>,
    pub accel_bias: na::Vector3<f32>,
}

impl IMU {
    pub fn new(gyro_noise: f32, accel_noise: f32) -> Self {
        Self {
            gyro_noise_std: gyro_noise,
            accel_noise_std: accel_noise,
            sampling_rate: 1000.0,
            gyro_bias: na::Vector3::zeros(),
            accel_bias: na::Vector3::zeros(),
        }
    }

    pub fn read_angular_velocity(
        &self,
        true_value: na::Vector3<f32>,
        rng: &mut impl rand::Rng,
    ) -> na::Vector3<f32> {
        let noise_dist = Normal::new(0.0, self.gyro_noise_std).unwrap();
        let noise = na::Vector3::new(
            noise_dist.sample(rng),
            noise_dist.sample(rng),
            noise_dist.sample(rng),
        );

        true_value + self.gyro_bias + noise
    }

    pub fn read_linear_acceleration(
        &self,
        true_value: na::Vector3<f32>,
        rng: &mut impl rand::Rng,
    ) -> na::Vector3<f32> {
        let noise_dist = Normal::new(0.0, self.accel_noise_std).unwrap();
        let noise = na::Vector3::new(
            noise_dist.sample(rng),
            noise_dist.sample(rng),
            noise_dist.sample(rng),
        );

        true_value + self.accel_bias + noise
    }
}

/// Joint encoder
#[derive(Debug, Clone)]
pub struct Encoder {
    pub resolution: u32,
    pub noise_std: f32,
}

impl Encoder {
    pub fn new(resolution: u32, noise: f32) -> Self {
        Self {
            resolution,
            noise_std: noise,
        }
    }

    pub fn read_position(
        &self,
        true_position: f32,
        rng: &mut impl rand::Rng,
    ) -> f32 {
        let noise_dist = Normal::new(0.0, self.noise_std).unwrap();
        let noise = noise_dist.sample(rng);

        let counts_per_rad = self.resolution as f32 / (2.0 * std::f32::consts::PI);
        let quantized = (true_position * counts_per_rad).round() / counts_per_rad;

        quantized + noise
    }
}

/// Contact sensor (force-sensitive resistor)
#[derive(Debug, Clone)]
pub struct ContactSensor {
    pub range: (f32, f32),
    pub noise_std: f32,
}

impl ContactSensor {
    pub fn new(range: (f32, f32), noise: f32) -> Self {
        Self { range, noise_std: noise }
    }

    pub fn read_force(
        &self,
        true_force: f32,
        rng: &mut impl rand::Rng,
    ) -> f32 {
        let noise_dist = Normal::new(0.0, self.noise_std).unwrap();
        let noise = noise_dist.sample(rng);

        (true_force + noise).clamp(self.range.0, self.range.1)
    }

    pub fn is_contact(&self, force: f32) -> bool {
        force > 5.0
    }
}
