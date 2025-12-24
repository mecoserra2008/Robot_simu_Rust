use crate::actuator::{Actuator, ActuatorSpec};
use crate::sensor::{IMU, Encoder, ContactSensor};
use physics::{PhysicsEngine, BodyHandle, na, RigidBodyBuilder, ColliderBuilder, vector};
use serde::Deserialize;

#[derive(Debug, Clone)]
pub struct Leg {
    pub hip_x: Actuator,
    pub hip_y: Actuator,
    pub knee: Actuator,
    pub gripper: Gripper,
    pub contact_sensor: ContactSensor,
    pub hip_body: BodyHandle,
    pub thigh_body: BodyHandle,
    pub shin_body: BodyHandle,
}

#[derive(Debug, Clone)]
pub struct Head {
    pub pitch: Actuator,  // Up/down
    pub yaw: Actuator,    // Left/right
    pub body: BodyHandle,
}

#[derive(Debug, Clone)]
pub struct Gripper {
    pub open_close: Actuator,
    pub is_grasping: bool,
    pub grasped_object: Option<usize>,
    pub grip_force: f32,
}

pub struct QuadrupedRobot {
    pub base_body: BodyHandle,
    pub head: Head,
    pub legs: [Leg; 4],
    pub imu: IMU,
    pub encoders: Vec<Encoder>,
    pub body_mass: f32,
    pub body_dimensions: na::Vector3<f32>,
    pub health: f32,
    pub max_health: f32,
}

impl QuadrupedRobot {
    pub fn new(physics: &mut PhysicsEngine, spec: &RobotConfig) -> Self {
        let base_body = Self::create_base_body(physics, spec);
        let head = Self::create_head(physics, base_body, spec);
        let legs = [
            Self::create_leg(physics, base_body, spec, 0),
            Self::create_leg(physics, base_body, spec, 1),
            Self::create_leg(physics, base_body, spec, 2),
            Self::create_leg(physics, base_body, spec, 3),
        ];

        Self {
            base_body,
            head,
            legs,
            imu: IMU::new(0.014, 0.15),
            encoders: vec![Encoder::new(8192, 0.0005); 16],  // 12 legs + 2 head + 2 grippers
            body_mass: spec.body.mass,
            body_dimensions: na::Vector3::from_row_slice(&spec.body.dimensions),
            health: 100.0,
            max_health: 100.0,
        }
    }

    fn create_head(physics: &mut PhysicsEngine, _base_body: BodyHandle, _spec: &RobotConfig) -> Head {
        let head_body = physics.create_rigid_body(
            RigidBodyBuilder::dynamic()
                .translation(vector![0.0, 0.4, 0.3])  // In front and above base
                .build()
        );

        let head_collider = ColliderBuilder::ball(0.1)
            .density(0.5)
            .friction(0.3)
            .build();
        physics.create_collider(head_collider, head_body);

        let pitch_spec = ActuatorSpec {
            continuous_torque: 5.0,
            peak_torque: 10.0,
            max_velocity: 20.0,
            rotor_inertia: 0.00001,
            gear_ratio: 6.0,
            motor_constant: 0.05,
            resistance: 0.03,
            damping: 0.3,
            friction: 0.05,
            backlash: 0.01,
            position_limit: (-1.0, 1.0),  // ~±57 degrees
        };

        Head {
            pitch: Actuator::new(pitch_spec.clone()),
            yaw: Actuator::new(ActuatorSpec {
                position_limit: (-1.5, 1.5),  // ~±86 degrees
                ..pitch_spec
            }),
            body: head_body,
        }
    }

    fn create_base_body(physics: &mut PhysicsEngine, spec: &RobotConfig) -> BodyHandle {
        let rb = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, 0.5, 0.0])
            .build();
        let handle = physics.create_rigid_body(rb);

        let collider = ColliderBuilder::cuboid(
            spec.body.dimensions[0] / 2.0,
            spec.body.dimensions[1] / 2.0,
            spec.body.dimensions[2] / 2.0,
        )
        .density(spec.body.mass / (spec.body.dimensions.iter().product::<f32>()))
        .friction(spec.body.surface_friction)
        .restitution(spec.body.restitution)
        .build();

        physics.create_collider(collider, handle);
        handle
    }

    fn create_leg(physics: &mut PhysicsEngine, _base: BodyHandle, spec: &RobotConfig, idx: usize) -> Leg {
        let pos = &spec.legs.positions[idx];

        let make_spec = |limits: &[f32]| ActuatorSpec {
            continuous_torque: spec.actuators.continuous_torque,
            peak_torque: spec.actuators.peak_torque,
            max_velocity: spec.actuators.max_velocity,
            rotor_inertia: spec.actuators.rotor_inertia,
            gear_ratio: spec.actuators.gear_ratio,
            motor_constant: spec.actuators.motor_constant,
            resistance: spec.actuators.resistance,
            damping: spec.actuators.damping,
            friction: spec.actuators.friction,
            backlash: spec.actuators.backlash.to_radians(),
            position_limit: (limits[0], limits[1]),
        };

        let hip_x = Actuator::new(make_spec(&spec.actuators.limits.hip_x));
        let hip_y = Actuator::new(make_spec(&spec.actuators.limits.hip_y));
        let knee = Actuator::new(make_spec(&spec.actuators.limits.knee));

        let hip_body = physics.create_rigid_body(
            RigidBodyBuilder::dynamic().translation(vector![pos[0], pos[1], pos[2]]).build()
        );
        let thigh_body = physics.create_rigid_body(RigidBodyBuilder::dynamic().build());
        let shin_body = physics.create_rigid_body(RigidBodyBuilder::dynamic().build());

        let gripper_spec = ActuatorSpec {
            continuous_torque: 3.0,
            peak_torque: 6.0,
            max_velocity: 10.0,
            rotor_inertia: 0.00001,
            gear_ratio: 4.0,
            motor_constant: 0.03,
            resistance: 0.02,
            damping: 0.2,
            friction: 0.03,
            backlash: 0.005,
            position_limit: (0.0, 0.8),  // 0 = open, 0.8 = closed
        };

        Leg {
            hip_x,
            hip_y,
            knee,
            gripper: Gripper {
                open_close: Actuator::new(gripper_spec),
                is_grasping: false,
                grasped_object: None,
                grip_force: 0.0,
            },
            contact_sensor: ContactSensor::new((0.0, 100.0), 2.0),
            hip_body,
            thigh_body,
            shin_body,
        }
    }

    pub fn get_observation(&self, physics: &PhysicsEngine, rng: &mut impl rand::Rng) -> Vec<f32> {
        let mut obs = Vec::with_capacity(48);

        if let Some(body) = physics.get_body(self.base_body) {
            let pos = body.translation();
            let rot = body.rotation().quaternion();
            let linvel = body.linvel();
            let angvel = body.angvel();

            obs.extend_from_slice(&[pos.x, pos.y, pos.z]);
            obs.extend_from_slice(&[rot.w, rot.i, rot.j, rot.k]);
            obs.extend_from_slice(&[linvel.x, linvel.y, linvel.z]);

            let angvel_noisy = self.imu.read_angular_velocity(
                na::Vector3::new(angvel.x, angvel.y, angvel.z),
                rng,
            );
            obs.extend_from_slice(angvel_noisy.as_slice());
        }

        for (i, leg) in self.legs.iter().enumerate() {
            let base = i * 3;
            obs.push(self.encoders[base].read_position(leg.hip_x.state.position, rng));
            obs.push(leg.hip_x.state.velocity);
            obs.push(self.encoders[base + 1].read_position(leg.hip_y.state.position, rng));
            obs.push(leg.hip_y.state.velocity);
            obs.push(self.encoders[base + 2].read_position(leg.knee.state.position, rng));
            obs.push(leg.knee.state.velocity);
        }

        for leg in &self.legs {
            obs.push(if leg.contact_sensor.is_contact(0.0) { 1.0 } else { 0.0 });
        }

        obs
    }

    pub fn apply_action(&mut self, action: &[f32]) {
        for (i, leg) in self.legs.iter_mut().enumerate() {
            let base = i * 3;
            leg.hip_x.set_position_target(action[base]);
            leg.hip_y.set_position_target(action[base + 1]);
            leg.knee.set_position_target(action[base + 2]);
        }
    }

    pub fn update_actuators(&mut self, dt: f32) {
        for leg in &mut self.legs {
            leg.hip_x.update(dt);
            leg.hip_y.update(dt);
            leg.knee.update(dt);
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct RobotConfig {
    pub body: BodyConfig,
    pub legs: LegsConfig,
    pub actuators: ActuatorConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct BodyConfig {
    pub mass: f32,
    pub dimensions: Vec<f32>,
    pub surface_friction: f32,
    pub restitution: f32,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LegsConfig {
    pub positions: Vec<Vec<f32>>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ActuatorConfig {
    pub continuous_torque: f32,
    pub peak_torque: f32,
    pub max_velocity: f32,
    pub rotor_inertia: f32,
    pub gear_ratio: f32,
    pub motor_constant: f32,
    pub resistance: f32,
    pub damping: f32,
    pub friction: f32,
    pub backlash: f32,
    pub limits: ActuatorLimits,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ActuatorLimits {
    pub hip_x: Vec<f32>,
    pub hip_y: Vec<f32>,
    pub knee: Vec<f32>,
}
