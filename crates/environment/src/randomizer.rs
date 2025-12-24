use physics::PhysicsEngine;
use rand::Rng;
use rand_distr::{Distribution, Uniform};

pub struct DomainRandomizer {
    pub mass_variation: f32,
    pub friction_variation: f32,
    pub latency_range: (f32, f32),
}

impl Default for DomainRandomizer {
    fn default() -> Self {
        Self {
            mass_variation: 0.1,
            friction_variation: 0.3,
            latency_range: (0.5, 2.0),
        }
    }
}

impl DomainRandomizer {
    pub fn randomize_physics(&self, physics: &mut PhysicsEngine, rng: &mut impl Rng) {
        let gravity_var = Uniform::new(-0.5, 0.5);
        physics.gravity.y = -9.81 + gravity_var.sample(rng);

        for (_, collider) in physics.collider_set.iter_mut() {
            let nominal_friction = 0.8;
            let friction_dist = Uniform::new(
                nominal_friction * (1.0 - self.friction_variation),
                nominal_friction * (1.0 + self.friction_variation),
            );
            collider.set_friction(friction_dist.sample(rng));
        }
    }

    pub fn get_sensor_latency(&self, rng: &mut impl Rng) -> f32 {
        let latency_dist = Uniform::new(self.latency_range.0, self.latency_range.1);
        latency_dist.sample(rng) / 1000.0
    }

    pub fn apply_external_disturbance(
        &self,
        physics: &mut PhysicsEngine,
        body_handle: physics::BodyHandle,
        rng: &mut impl Rng,
    ) {
        if rng.gen::<f32>() < 0.1 {
            let force_dist = Uniform::new(10.0, 50.0);
            let force_mag = force_dist.sample(rng);

            let angle = rng.gen_range(0.0..std::f32::consts::TAU);
            let force = physics::na::Vector3::new(
                force_mag * angle.cos(),
                0.0,
                force_mag * angle.sin(),
            );

            physics.apply_force(body_handle, force);
        }
    }
}
