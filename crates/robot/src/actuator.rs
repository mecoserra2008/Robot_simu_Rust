use serde::{Deserialize, Serialize};

/// Motor specifications
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorSpec {
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
    pub position_limit: (f32, f32),
}

/// Motor state
#[derive(Debug, Clone)]
pub struct ActuatorState {
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub current: f32,
    pub temperature: f32,
}

/// PD Controller for position control
#[derive(Debug, Clone)]
pub struct PDController {
    pub kp: f32,
    pub kd: f32,
}

impl PDController {
    pub fn new(kp: f32, kd: f32) -> Self {
        Self { kp, kd }
    }

    pub fn compute_torque(
        &self,
        target_position: f32,
        current_position: f32,
        current_velocity: f32,
    ) -> f32 {
        let pos_error = target_position - current_position;
        let vel_error = 0.0 - current_velocity;

        self.kp * pos_error + self.kd * vel_error
    }
}

/// Complete actuator model
#[derive(Debug, Clone)]
pub struct Actuator {
    pub spec: ActuatorSpec,
    pub state: ActuatorState,
    pub controller: PDController,
    pub target_position: f32,
}

impl Actuator {
    pub fn new(spec: ActuatorSpec) -> Self {
        Self {
            spec: spec.clone(),
            state: ActuatorState {
                position: 0.0,
                velocity: 0.0,
                torque: 0.0,
                current: 0.0,
                temperature: 25.0,
            },
            controller: PDController::new(100.0, 10.0),
            target_position: 0.0,
        }
    }

    pub fn update(&mut self, dt: f32) {
        let desired_torque = self.controller.compute_torque(
            self.target_position,
            self.state.position,
            self.state.velocity,
        );

        let limited_torque = desired_torque.clamp(
            -self.spec.peak_torque,
            self.spec.peak_torque,
        );

        let friction_torque = self.spec.friction * self.state.velocity.signum();
        let damping_torque = self.spec.damping * self.state.velocity;

        let total_torque = limited_torque - friction_torque - damping_torque;

        self.state.current = total_torque / self.spec.motor_constant;

        let power_loss = self.state.current.powi(2) * self.spec.resistance;
        self.state.temperature += power_loss * dt * 0.01;
        self.state.temperature -= (self.state.temperature - 25.0) * dt * 0.05;

        self.state.torque = total_torque;
    }

    pub fn set_position_target(&mut self, position: f32) {
        self.target_position = position.clamp(
            self.spec.position_limit.0,
            self.spec.position_limit.1,
        );
    }
}
