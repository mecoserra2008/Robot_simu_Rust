# Complete Robotics RL Simulation in Rust - Implementation Guide

## 📋 Table of Contents

1. [Project Overview](#project-overview)
2. [System Requirements](#system-requirements)
3. [Project Structure](#project-structure)
4. [Phase 1: Setup & Dependencies](#phase-1-setup--dependencies)
5. [Phase 2: Physics Engine Foundation](#phase-2-physics-engine-foundation)
6. [Phase 3: Robot Specification](#phase-3-robot-specification)
7. [Phase 4: Environment Objects](#phase-4-environment-objects)
8. [Phase 5: Simulation Core](#phase-5-simulation-core)
9. [Phase 6: Reinforcement Learning](#phase-6-reinforcement-learning)
10. [Phase 7: Visualization](#phase-7-visualization)
11. [Phase 8: Training Pipeline](#phase-8-training-pipeline)
12. [Complete Code Implementation](#complete-code-implementation)

---

## Project Overview

**Goal**: Build a complete robotics simulation where a quadruped robot learns to walk, navigate, and interact with dynamic objects using Reinforcement Learning (PPO algorithm), all implemented in Rust.

**Architecture**:
```
┌─────────────────────────────────────────────┐
│         Visualization Layer (Bevy)          │
│  ┌──────────────┐        ┌──────────────┐  │
│  │ 3D Renderer  │        │  UI/Metrics  │  │
│  └──────────────┘        └──────────────┘  │
└─────────────────────────────────────────────┘
                    ↕
┌─────────────────────────────────────────────┐
│      RL Training Layer (PPO in Rust)        │
│  ┌──────────────┐        ┌──────────────┐  │
│  │ Policy Net   │◄───────┤ Replay Buffer│  │
│  └──────────────┘        └──────────────┘  │
└─────────────────────────────────────────────┘
                    ↕
┌─────────────────────────────────────────────┐
│    Simulation Layer (Rapier Physics)        │
│  ┌──────────────┐  ┌──────────────────────┐│
│  │ Robot Model  │  │ Environment Objects  ││
│  └──────────────┘  └──────────────────────┘│
└─────────────────────────────────────────────┘
```

**Key Features**:
- ✅ Full physics simulation (rigid body dynamics, collisions, friction)
- ✅ Quadruped robot with 12 actuated joints
- ✅ Dynamic environment (moving humans, animals, obstacles)
- ✅ Domain randomization (mass, friction, latency variations)
- ✅ PPO reinforcement learning algorithm
- ✅ Real-time 3D visualization
- ✅ Comprehensive reward shaping
- ✅ Multi-threaded training (parallel environments)

---

## System Requirements

### Hardware
- **CPU**: 8+ cores (Ryzen 7 / Intel i7 or better)
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: Optional (OpenGL 4.5+ for visualization)
- **Storage**: 2GB free space

### Software
- **Rust**: 1.75+ (latest stable)
- **OS**: Linux (Ubuntu 22.04+), macOS, or Windows 10+
- **Tools**: `cargo`, `git`, `rustup`

### Rust Knowledge Level
- **Required**: Intermediate (ownership, traits, async)
- **Helpful**: Basic linear algebra, control theory

---

## Project Structure

```
quadruped_rl/
├── Cargo.toml                 # Root manifest
├── README.md
├── LICENSE
│
├── crates/
│   ├── physics/               # Physics engine wrapper
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── engine.rs      # Rapier integration
│   │       ├── rigid_body.rs
│   │       └── collision.rs
│   │
│   ├── robot/                 # Robot definition
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── quadruped.rs   # Robot structure
│   │       ├── actuator.rs    # Motor models
│   │       └── sensor.rs      # IMU, encoders
│   │
│   ├── environment/           # World & objects
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── world.rs       # Scene management
│   │       ├── objects.rs     # Obstacles, terrain
│   │       └── randomizer.rs  # Domain randomization
│   │
│   ├── rl/                    # Reinforcement learning
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── ppo.rs         # PPO algorithm
│   │       ├── network.rs     # Neural network
│   │       ├── replay.rs      # Experience buffer
│   │       └── reward.rs      # Reward function
│   │
│   └── visualization/         # 3D rendering
│       ├── Cargo.toml
│       └── src/
│           ├── lib.rs
│           ├── renderer.rs    # Bevy integration
│           └── ui.rs          # Metrics display
│
├── config/
│   ├── robot_spec.toml        # Robot parameters
│   ├── training.toml          # Hyperparameters
│   └── objects.toml           # Environment objects
│
└── src/
    └── main.rs                # Entry point
```

---

## Phase 1: Setup & Dependencies

### Step 1.1: Initialize Project

```bash
# Create workspace
cargo new --bin quadruped_rl
cd quadruped_rl

# Create workspace structure
mkdir -p crates/{physics,robot,environment,rl,visualization}/src
mkdir -p config
```

### Step 1.2: Root Cargo.toml

```toml
[workspace]
members = [
    "crates/physics",
    "crates/robot",
    "crates/environment",
    "crates/rl",
    "crates/visualization",
]

[package]
name = "quadruped_rl"
version = "0.1.0"
edition = "2021"

[dependencies]
# Workspace crates
physics = { path = "crates/physics" }
robot = { path = "crates/robot" }
environment = { path = "crates/environment" }
rl = { path = "crates/rl" }
visualization = { path = "crates/visualization" }

# Core dependencies
nalgebra = "0.32"           # Linear algebra
rand = "0.8"                # Random number generation
serde = { version = "1.0", features = ["derive"] }
toml = "0.8"                # Config parsing
anyhow = "1.0"              # Error handling
rayon = "1.8"               # Parallel processing
crossbeam = "0.8"           # Lock-free primitives

# Logging
tracing = "0.1"
tracing-subscriber = "0.3"

[profile.release]
opt-level = 3
lto = "fat"
codegen-units = 1
```

### Step 1.3: Individual Crate Manifests

**`crates/physics/Cargo.toml`**:
```toml
[package]
name = "physics"
version = "0.1.0"
edition = "2021"

[dependencies]
rapier3d = { version = "0.17", features = ["parallel", "simd-stable"] }
nalgebra = "0.32"
serde = { version = "1.0", features = ["derive"] }
```

**`crates/robot/Cargo.toml`**:
```toml
[package]
name = "robot"
version = "0.1.0"
edition = "2021"

[dependencies]
physics = { path = "../physics" }
nalgebra = "0.32"
serde = { version = "1.0", features = ["derive"] }
rand = "0.8"
```

**`crates/environment/Cargo.toml`**:
```toml
[package]
name = "environment"
version = "0.1.0"
edition = "2021"

[dependencies]
physics = { path = "../physics" }
robot = { path = "../robot" }
nalgebra = "0.32"
serde = { version = "1.0", features = ["derive"] }
rand = "0.8"
rand_distr = "0.4"
```

**`crates/rl/Cargo.toml`**:
```toml
[package]
name = "rl"
version = "0.1.0"
edition = "2021"

[dependencies]
environment = { path = "../environment" }
nalgebra = "0.32"
ndarray = { version = "0.15", features = ["rayon"] }
rand = "0.8"
rand_distr = "0.4"
rayon = "1.8"
serde = { version = "1.0", features = ["derive"] }

# Neural network
burn = { version = "0.13", features = ["ndarray"] }
burn-ndarray = "0.13"
```

**`crates/visualization/Cargo.toml`**:
```toml
[package]
name = "visualization"
version = "0.1.0"
edition = "2021"

[dependencies]
robot = { path = "../robot" }
environment = { path = "../environment" }
bevy = { version = "0.12", features = ["dynamic_linking"] }
bevy_rapier3d = "0.23"
nalgebra = "0.32"
```

---

## Phase 2: Physics Engine Foundation

### Step 2.1: Physics Engine Wrapper (`crates/physics/src/engine.rs`)

```rust
use rapier3d::prelude::*;
use nalgebra as na;
use std::collections::HashMap;

/// Handle to a physics object
pub type BodyHandle = RigidBodyHandle;
pub type ColliderHandle = rapier3d::geometry::ColliderHandle;

/// Main physics engine
pub struct PhysicsEngine {
    /// Integration parameters
    pub integration_params: IntegrationParameters,
    
    /// Physics pipeline
    pub physics_pipeline: PhysicsPipeline,
    
    /// Island manager
    pub island_manager: IslandManager,
    
    /// Broad phase
    pub broad_phase: BroadPhase,
    
    /// Narrow phase
    pub narrow_phase: NarrowPhase,
    
    /// Rigid bodies
    pub rigid_body_set: RigidBodySet,
    
    /// Colliders
    pub collider_set: ColliderSet,
    
    /// Impulse joints
    pub impulse_joint_set: ImpulseJointSet,
    
    /// Multibody joints
    pub multibody_joint_set: MultibodyJointSet,
    
    /// CCD solver
    pub ccd_solver: CCDSolver,
    
    /// Query pipeline
    pub query_pipeline: QueryPipeline,
    
    /// Gravity vector
    pub gravity: na::Vector3<f32>,
    
    /// Time step (seconds)
    pub dt: f32,
    
    /// Current simulation time
    pub time: f32,
}

impl PhysicsEngine {
    pub fn new(dt: f32, gravity: na::Vector3<f32>) -> Self {
        let mut integration_params = IntegrationParameters::default();
        integration_params.dt = dt;
        
        Self {
            integration_params,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            gravity,
            dt,
            time: 0.0,
        }
    }
    
    /// Step the simulation forward
    pub fn step(&mut self) {
        let gravity = self.gravity;
        let integration_params = self.integration_params;
        
        self.physics_pipeline.step(
            &gravity,
            &integration_params,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None, // No query pipeline hooks
            &(),  // No physics hooks
            &(),  // No events
        );
        
        self.query_pipeline.update(&self.rigid_body_set, &self.collider_set);
        self.time += self.dt;
    }
    
    /// Create a rigid body
    pub fn create_rigid_body(&mut self, rb: RigidBody) -> BodyHandle {
        self.rigid_body_set.insert(rb)
    }
    
    /// Create a collider attached to a body
    pub fn create_collider(
        &mut self,
        collider: Collider,
        parent: BodyHandle,
    ) -> ColliderHandle {
        self.collider_set.insert_with_parent(collider, parent, &mut self.rigid_body_set)
    }
    
    /// Get rigid body
    pub fn get_body(&self, handle: BodyHandle) -> Option<&RigidBody> {
        self.rigid_body_set.get(handle)
    }
    
    /// Get mutable rigid body
    pub fn get_body_mut(&mut self, handle: BodyHandle) -> Option<&mut RigidBody> {
        self.rigid_body_set.get_mut(handle)
    }
    
    /// Apply force to body
    pub fn apply_force(&mut self, handle: BodyHandle, force: na::Vector3<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.add_force(force, true);
        }
    }
    
    /// Apply torque to body
    pub fn apply_torque(&mut self, handle: BodyHandle, torque: na::Vector3<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.add_torque(torque, true);
        }
    }
    
    /// Reset forces on all bodies
    pub fn reset_forces(&mut self) {
        for (_, body) in self.rigid_body_set.iter_mut() {
            body.reset_forces(true);
            body.reset_torques(true);
        }
    }
}
```

### Step 2.2: Physics Library Entry Point (`crates/physics/src/lib.rs`)

```rust
pub mod engine;

pub use engine::{PhysicsEngine, BodyHandle, ColliderHandle};
pub use rapier3d::prelude::*;
pub use nalgebra as na;
```

---

## Phase 3: Robot Specification

### Step 3.1: Robot Configuration (`config/robot_spec.toml`)

```toml
[robot]
name = "Quadruped-v1"
type = "quadruped"
total_mass = 20.3  # kg

[body]
mass = 8.5  # kg
dimensions = [0.6, 0.3, 0.2]  # L×W×H meters
center_of_mass = [0.0, 0.0, 0.05]
moment_of_inertia = [
    [0.15, 0.0, 0.0],
    [0.0, 0.35, 0.0],
    [0.0, 0.0, 0.45]
]
surface_friction = 0.6
restitution = 0.1

[legs]
count = 4
positions = [
    [0.25, 0.15, 0.0],   # Front left
    [0.25, -0.15, 0.0],  # Front right
    [-0.25, 0.15, 0.0],  # Rear left
    [-0.25, -0.15, 0.0], # Rear right
]

[legs.segments]
names = ["hip", "thigh", "shin"]
masses = [0.8, 1.2, 0.6]  # kg
lengths = [0.15, 0.25, 0.25]  # meters
radii = [0.03, 0.025, 0.02]  # meters

[actuators]
count = 12  # 3 per leg
type = "T-Motor_AK80-9"
continuous_torque = 9.0   # N⋅m
peak_torque = 18.0        # N⋅m
max_velocity = 38.2       # rad/s
rotor_inertia = 0.000025  # kg⋅m²
gear_ratio = 9.0
backlash = 0.05           # degrees
motor_constant = 0.091    # N⋅m/A
resistance = 0.055        # Ohms
damping = 0.5            # N⋅m⋅s/rad
friction = 0.1           # N⋅m

[actuators.limits]
hip_x = [-0.8, 0.8]      # radians
hip_y = [-1.2, 1.2]
knee = [-2.4, -0.5]

[sensors.imu]
gyro_noise = 0.014      # rad/s stddev
accel_noise = 0.15      # m/s² stddev
sampling_rate = 1000    # Hz
bias_drift = 0.001      # rad/s per hour

[sensors.encoders]
resolution = 8192       # counts/rev
noise = 0.0005         # radians stddev
latency = 0.5          # ms

[sensors.contact]
range = [0.0, 100.0]   # Newtons
noise = 2.0            # N stddev
```

### Step 3.2: Actuator Model (`crates/robot/src/actuator.rs`)

```rust
use nalgebra as na;
use serde::{Deserialize, Serialize};

/// Motor specifications
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorSpec {
    pub continuous_torque: f32,  // N⋅m
    pub peak_torque: f32,
    pub max_velocity: f32,       // rad/s
    pub rotor_inertia: f32,      // kg⋅m²
    pub gear_ratio: f32,
    pub motor_constant: f32,     // N⋅m/A
    pub resistance: f32,         // Ohms
    pub damping: f32,            // N⋅m⋅s/rad
    pub friction: f32,           // N⋅m
    pub backlash: f32,           // radians
    pub position_limit: (f32, f32), // (min, max) radians
}

/// Motor state
#[derive(Debug, Clone)]
pub struct ActuatorState {
    pub position: f32,      // radians
    pub velocity: f32,      // rad/s
    pub torque: f32,        // N⋅m (current output)
    pub current: f32,       // Amperes
    pub temperature: f32,   // Celsius
}

/// PD Controller for position control
#[derive(Debug, Clone)]
pub struct PDController {
    pub kp: f32,  // Position gain
    pub kd: f32,  // Velocity gain
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
        let vel_error = 0.0 - current_velocity; // Target velocity = 0
        
        self.kp * pos_error + self.kd * vel_error
    }
}

/// Complete actuator model
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
    
    /// Update actuator state (call at 1kHz)
    pub fn update(&mut self, dt: f32) {
        // Compute torque from PD controller
        let desired_torque = self.controller.compute_torque(
            self.target_position,
            self.state.position,
            self.state.velocity,
        );
        
        // Apply limits
        let limited_torque = desired_torque.clamp(
            -self.spec.peak_torque,
            self.spec.peak_torque,
        );
        
        // Model friction and damping
        let friction_torque = self.spec.friction * self.state.velocity.signum();
        let damping_torque = self.spec.damping * self.state.velocity;
        
        let total_torque = limited_torque - friction_torque - damping_torque;
        
        // Calculate current
        self.state.current = total_torque / self.spec.motor_constant;
        
        // Heat model (simplified)
        let power_loss = self.state.current.powi(2) * self.spec.resistance;
        self.state.temperature += power_loss * dt * 0.01; // Simplified thermal model
        self.state.temperature -= (self.state.temperature - 25.0) * dt * 0.05; // Cooling
        
        self.state.torque = total_torque;
    }
    
    pub fn set_position_target(&mut self, position: f32) {
        self.target_position = position.clamp(
            self.spec.position_limit.0,
            self.spec.position_limit.1,
        );
    }
}
```

### Step 3.3: Sensor Models (`crates/robot/src/sensor.rs`)

```rust
use nalgebra as na;
use rand_distr::{Distribution, Normal};

/// IMU sensor (gyroscope + accelerometer)
pub struct IMU {
    pub gyro_noise_std: f32,      // rad/s
    pub accel_noise_std: f32,     // m/s²
    pub sampling_rate: f32,       // Hz
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
pub struct Encoder {
    pub resolution: u32,     // counts per revolution
    pub noise_std: f32,      // radians
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
        
        // Quantization
        let counts_per_rad = self.resolution as f32 / (2.0 * std::f32::consts::PI);
        let quantized = (true_position * counts_per_rad).round() / counts_per_rad;
        
        quantized + noise
    }
}

/// Contact sensor (force-sensitive resistor)
pub struct ContactSensor {
    pub range: (f32, f32),  // (min, max) Newtons
    pub noise_std: f32,     // N
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
        force > 5.0  // Threshold in Newtons
    }
}
```

### Step 3.4: Quadruped Robot (`crates/robot/src/quadruped.rs`)

```rust
use crate::actuator::{Actuator, ActuatorSpec};
use crate::sensor::{IMU, Encoder, ContactSensor};
use physics::{PhysicsEngine, BodyHandle, na};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};

/// Leg configuration
#[derive(Debug, Clone)]
pub struct Leg {
    pub hip_x: Actuator,
    pub hip_y: Actuator,
    pub knee: Actuator,
    pub contact_sensor: ContactSensor,
    
    // Physics bodies
    pub hip_body: BodyHandle,
    pub thigh_body: BodyHandle,
    pub shin_body: BodyHandle,
}

/// Complete quadruped robot
pub struct QuadrupedRobot {
    pub base_body: BodyHandle,
    pub legs: [Leg; 4],
    pub imu: IMU,
    pub encoders: Vec<Encoder>,
    
    // Robot spec
    pub body_mass: f32,
    pub body_dimensions: na::Vector3<f32>,
}

impl QuadrupedRobot {
    pub fn new(physics: &mut PhysicsEngine, spec: &RobotConfig) -> Self {
        // Create base body
        let base_body = Self::create_base_body(physics, &spec);
        
        // Create legs
        let legs = [
            Self::create_leg(physics, base_body, spec, 0),
            Self::create_leg(physics, base_body, spec, 1),
            Self::create_leg(physics, base_body, spec, 2),
            Self::create_leg(physics, base_body, spec, 3),
        ];
        
        // Create sensors
        let imu = IMU::new(0.014, 0.15);
        let encoders = vec![Encoder::new(8192, 0.0005); 12];
        
        Self {
            base_body,
            legs,
            imu,
            encoders,
            body_mass: spec.body.mass,
            body_dimensions: na::Vector3::from_row_slice(&spec.body.dimensions),
        }
    }
    
    fn create_base_body(
        physics: &mut PhysicsEngine,
        spec: &RobotConfig,
    ) -> BodyHandle {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, 0.5, 0.0])  // Start 0.5m above ground
            .build();
        
        let handle = physics.create_rigid_body(rigid_body);
        
        // Create collider
        let collider = ColliderBuilder::cuboid(
            spec.body.dimensions[0] / 2.0,
            spec.body.dimensions[1] / 2.0,
            spec.body.dimensions[2] / 2.0,
        )
        .density(spec.body.mass / (spec.body.dimensions[0] * spec.body.dimensions[1] * spec.body.dimensions[2]))
        .friction(spec.body.surface_friction)
        .restitution(spec.body.restitution)
        .build();
        
        physics.create_collider(collider, handle);
        
        handle
    }
    
    fn create_leg(
        physics: &mut PhysicsEngine,
        base_body: BodyHandle,
        spec: &RobotConfig,
        leg_index: usize,
    ) -> Leg {
        let leg_pos = &spec.legs.positions[leg_index];
        
        // Create actuators
        let hip_x = Actuator::new(ActuatorSpec {
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
            position_limit: (
                spec.actuators.limits.hip_x[0],
                spec.actuators.limits.hip_x[1],
            ),
        });
        
        let hip_y = Actuator::new(ActuatorSpec {
            position_limit: (
                spec.actuators.limits.hip_y[0],
                spec.actuators.limits.hip_y[1],
            ),
            ..hip_x.spec.clone()
        });
        
        let knee = Actuator::new(ActuatorSpec {
            position_limit: (
                spec.actuators.limits.knee[0],
                spec.actuators.limits.knee[1],
            ),
            ..hip_x.spec.clone()
        });
        
        // Create bodies (simplified - in reality you'd add joints)
        let hip_body = physics.create_rigid_body(
            RigidBodyBuilder::dynamic()
                .translation(vector![leg_pos[0], leg_pos[1], leg_pos[2]])
                .build()
        );
        
        let thigh_body = physics.create_rigid_body(
            RigidBodyBuilder::dynamic().build()
        );
        
        let shin_body = physics.create_rigid_body(
            RigidBodyBuilder::dynamic().build()
        );
        
        Leg {
            hip_x,
            hip_y,
            knee,
            contact_sensor: ContactSensor::new((0.0, 100.0), 2.0),
            hip_body,
            thigh_body,
            shin_body,
        }
    }
    
    /// Get robot observation (state vector)
    pub fn get_observation(&self, physics: &PhysicsEngine, rng: &mut impl rand::Rng) -> Vec<f32> {
        let mut obs = Vec::with_capacity(48);
        
        // Base state
        if let Some(body) = physics.get_body(self.base_body) {
            let pos = body.translation();
            let rot = body.rotation().quaternion();
            let linvel = body.linvel();
            let angvel = body.angvel();
            
            obs.extend_from_slice(&[pos.x, pos.y, pos.z]);
            obs.extend_from_slice(&[rot.w, rot.i, rot.j, rot.k]);
            obs.extend_from_slice(&[linvel.x, linvel.y, linvel.z]);
            
            // Add IMU noise
            let angvel_noisy = self.imu.read_angular_velocity(
                na::Vector3::new(angvel.x, angvel.y, angvel.z),
                rng,
            );
            obs.extend_from_slice(angvel_noisy.as_slice());
        }
        
        // Joint states (12 joints × 2)
        for leg in &self.legs {
            let encoders = [
                &self.encoders[0],
                &self.encoders[1],
                &self.encoders[2],
            ];
            
            obs.push(encoders[0].read_position(leg.hip_x.state.position, rng));
            obs.push(leg.hip_x.state.velocity);
            obs.push(encoders[1].read_position(leg.hip_y.state.position, rng));
            obs.push(leg.hip_y.state.velocity);
            obs.push(encoders[2].read_position(leg.knee.state.position, rng));
            obs.push(leg.knee.state.velocity);
        }
        
        // Contact states (4 feet)
        for leg in &self.legs {
            obs.push(if leg.contact_sensor.is_contact(0.0) { 1.0 } else { 0.0 });
        }
        
        obs
    }
    
    /// Apply action (12D vector of target joint positions)
    pub fn apply_action(&mut self, action: &[f32]) {
        assert_eq!(action.len(), 12);
        
        for (i, leg) in self.legs.iter_mut().enumerate() {
            let base_idx = i * 3;
            leg.hip_x.set_position_target(action[base_idx]);
            leg.hip_y.set_position_target(action[base_idx + 1]);
            leg.knee.set_position_target(action[base_idx + 2]);
        }
    }
    
    /// Update all actuators
    pub fn update_actuators(&mut self, dt: f32) {
        for leg in &mut self.legs {
            leg.hip_x.update(dt);
            leg.hip_y.update(dt);
            leg.knee.update(dt);
        }
    }
}

// Configuration structures
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
    pub count: usize,
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
```

### Step 3.5: Robot Library (`crates/robot/src/lib.rs`)

```rust
pub mod actuator;
pub mod sensor;
pub mod quadruped;

pub use actuator::{Actuator, ActuatorSpec, PDController};
pub use sensor::{IMU, Encoder, ContactSensor};
pub use quadruped::{QuadrupedRobot, Leg, RobotConfig};
```

---

## Phase 4: Environment Objects

### Step 4.1: Object Definitions (`config/objects.toml`)

```toml
[[objects]]
id = "human_adult"
mass = 70.0
height = 1.75
width = 0.5
shape = "capsule"
friction = 0.7
restitution = 0.3
movement_pattern = "bipedal_walk"
speed_range = [0.8, 1.8]
unpredictability = 0.3

[[objects]]
id = "dog_medium"
mass = 25.0
dimensions = [0.8, 0.3, 0.5]
shape = "box"
friction = 0.8
restitution = 0.2
movement_pattern = "quadruped_trot"
speed_range = [1.5, 4.0]
unpredictability = 0.6

[[objects]]
id = "cardboard_box"
mass = 2.0
dimensions = [0.4, 0.4, 0.5]
shape = "box"
friction = 0.5
restitution = 0.1
compressibility = 0.05
fragility = 0.7

[[objects]]
id = "concrete_wall"
mass = "infinite"
dimensions = [10.0, 0.2, 3.0]
shape = "box"
friction = 0.9
restitution = 0.05
static = true

[[objects]]
id = "tree"
mass = 500.0
trunk_radius = 0.3
trunk_height = 5.0
shape = "cylinder"
friction = 0.8
restitution = 0.1
static = true
```

### Step 4.2: World and Objects (`crates/environment/src/world.rs`)

```rust
use physics::{PhysicsEngine, BodyHandle, na};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use rand::Rng;

#[derive(Debug, Clone, Deserialize)]
pub struct ObjectConfig {
    pub id: String,
    pub mass: MassValue,
    #[serde(default)]
    pub dimensions: Option<Vec<f32>>,
    #[serde(default)]
    pub height: Option<f32>,
    #[serde(default)]
    pub width: Option<f32>,
    pub shape: String,
    pub friction: f32,
    pub restitution: f32,
    #[serde(default)]
    pub static_obj: bool,
    #[serde(default)]
    pub movement_pattern: Option<String>,
    #[serde(default)]
    pub speed_range: Option<Vec<f32>>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum MassValue {
    Finite(f32),
    Infinite(String),  // "infinite"
}

pub struct DynamicObject {
    pub config: ObjectConfig,
    pub body_handle: BodyHandle,
    pub velocity_target: na::Vector3<f32>,
}

pub struct World {
    pub ground_handle: BodyHandle,
    pub objects: Vec<DynamicObject>,
    pub target_position: na::Vector3<f32>,
}

impl World {
    pub fn new(
        physics: &mut PhysicsEngine,
        object_configs: Vec<ObjectConfig>,
    ) -> Self {
        // Create ground
        let ground = RigidBodyBuilder::fixed()
            .translation(vector![0.0, 0.0, 0.0])
            .build();
        let ground_handle = physics.create_rigid_body(ground);
        
        let ground_collider = ColliderBuilder::cuboid(50.0, 0.1, 50.0)
            .friction(0.8)
            .restitution(0.1)
            .build();
        physics.create_collider(ground_collider, ground_handle);
        
        // Create objects
        let mut objects = Vec::new();
        for config in object_configs {
            if let Some(obj) = Self::create_object(physics, &config) {
                objects.push(obj);
            }
        }
        
        Self {
            ground_handle,
            objects,
            target_position: na::Vector3::new(5.0, 0.0, 0.0),
        }
    }
    
    fn create_object(
        physics: &mut PhysicsEngine,
        config: &ObjectConfig,
    ) -> Option<DynamicObject> {
        let is_static = config.static_obj;
        
        let rigid_body = if is_static {
            RigidBodyBuilder::fixed()
        } else {
            RigidBodyBuilder::dynamic()
        }
        .translation(vector![
            rand::thread_rng().gen_range(-10.0..10.0),
            0.5,
            rand::thread_rng().gen_range(-10.0..10.0)
        ])
        .build();
        
        let body_handle = physics.create_rigid_body(rigid_body);
        
        // Create collider based on shape
        let collider = match config.shape.as_str() {
            "box" => {
                let dims = config.dimensions.as_ref()?;
                ColliderBuilder::cuboid(dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0)
                    .friction(config.friction)
                    .restitution(config.restitution)
                    .build()
            }
            "capsule" => {
                let height = config.height?;
                let width = config.width?;
                ColliderBuilder::capsule_y(height / 2.0, width / 2.0)
                    .friction(config.friction)
                    .restitution(config.restitution)
                    .build()
            }
            "cylinder" => {
                let height = config.height?;
                ColliderBuilder::cylinder(height / 2.0, 0.3)
                    .friction(config.friction)
                    .restitution(config.restitution)
                    .build()
            }
            _ => return None,
        };
        
        physics.create_collider(collider, body_handle);
        
        Some(DynamicObject {
            config: config.clone(),
            body_handle,
            velocity_target: na::Vector3::zeros(),
        })
    }
    
    pub fn update(&mut self, physics: &mut PhysicsEngine, dt: f32) {
        // Update dynamic objects
        for obj in &mut self.objects {
            if let Some(ref pattern) = obj.config.movement_pattern {
                // Simple movement AI
                match pattern.as_str() {
                    "bipedal_walk" | "quadruped_trot" => {
                        // Random walk
                        if rand::thread_rng().gen::<f32>() < 0.01 {
                            let speed_range = obj.config.speed_range.as_ref().unwrap();
                            let speed = rand::thread_rng().gen_range(speed_range[0]..speed_range[1]);
                            let angle = rand::thread_rng().gen_range(0.0..std::f32::consts::TAU);
                            
                            obj.velocity_target = na::Vector3::new(
                                speed * angle.cos(),
                                0.0,
                                speed * angle.sin(),
                            );
                        }
                        
                        // Apply velocity
                        if let Some(body) = physics.get_body_mut(obj.body_handle) {
                            body.set_linvel(obj.velocity_target, true);
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}
```

### Step 4.3: Domain Randomization (`crates/environment/src/randomizer.rs`)

```rust
use physics::PhysicsEngine;
use rand::Rng;
use rand_distr::{Distribution, Normal, Uniform};

pub struct DomainRandomizer {
    pub mass_variation: f32,        // ±10%
    pub friction_variation: f32,    // ±30%
    pub latency_range: (f32, f32),  // 0.5-2.0 ms
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
    pub fn randomize_physics(
        &self,
        physics: &mut PhysicsEngine,
        rng: &mut impl Rng,
    ) {
        // Randomize gravity (slight variations)
        let gravity_var = Uniform::new(-0.5, 0.5);
        physics.gravity.y = -9.81 + gravity_var.sample(rng);
        
        // Randomize friction coefficients
        for (_, collider) in physics.collider_set.iter_mut() {
            let nominal_friction = 0.8; // Store this externally in practice
            let friction_dist = Uniform::new(
                nominal_friction * (1.0 - self.friction_variation),
                nominal_friction * (1.0 + self.friction_variation),
            );
            collider.set_friction(friction_dist.sample(rng));
        }
        
        // Randomize body masses
        for (_, body) in physics.rigid_body_set.iter_mut() {
            if !body.is_fixed() {
                let nominal_mass = body.mass(); // Store original mass
                let mass_dist = Uniform::new(
                    nominal_mass * (1.0 - self.mass_variation),
                    nominal_mass * (1.0 + self.mass_variation),
                );
                // Note: Rapier doesn't allow direct mass setting
                // You'd need to recreate bodies or use additional mass properties
            }
        }
    }
    
    pub fn get_sensor_latency(&self, rng: &mut impl Rng) -> f32 {
        let latency_dist = Uniform::new(self.latency_range.0, self.latency_range.1);
        latency_dist.sample(rng) / 1000.0  // Convert ms to seconds
    }
    
    pub fn apply_external_disturbance(
        &self,
        physics: &mut PhysicsEngine,
        body_handle: physics::BodyHandle,
        rng: &mut impl Rng,
    ) {
        if rng.gen::<f32>() < 0.1 {  // 10% chance per episode
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
```

### Step 4.4: Environment Library (`crates/environment/src/lib.rs`)

```rust
pub mod world;
pub mod randomizer;

pub use world::{World, DynamicObject, ObjectConfig};
pub use randomizer::DomainRandomizer;
```

---

## Phase 5: Simulation Core

I'll continue with the simulation environment integration in the next part. Would you like me to proceed with:
- Phase 5: Complete Simulation Environment
- Phase 6: PPO Implementation in Rust
- Phase 7: Visualization with Bevy
- Phase 8: Training Pipeline

Or would you like me to package everything we have so far into a complete, runnable skeleton project first?