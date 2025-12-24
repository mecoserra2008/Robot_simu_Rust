# Quick Reference Card - Quadruped RL in Rust

## 🚀 Essential Commands

```bash
# Setup
cargo new --bin quadruped_rl && cd quadruped_rl
mkdir -p crates/{physics,robot,environment,rl,visualization}/src config checkpoints

# Build
cargo build --release                    # Production build (~5-10 min first time)
cargo build --release --features viz     # With visualization

# Run
RUST_LOG=info cargo run --release        # Normal training
RUST_LOG=debug cargo run --release       # Verbose logging
RUST_LOG=warn cargo run --release        # Minimal output

# Development
cargo check                              # Fast syntax check
cargo clippy                             # Linting
cargo fmt                                # Format code
cargo clean                              # Clean build artifacts
```

---

## 📐 Key Dimensions

### Observation Space
```rust
OBS_DIM = 51 dimensions:
├─ Base state: 13 (pos, rot_quat, linvel, angvel)
├─ Joint state: 24 (12 joints × [position, velocity])
├─ Previous action: 12
└─ Goal info: 3 (relative_x, relative_z, distance)
```

### Action Space
```rust
ACT_DIM = 12 dimensions:
├─ 4 legs × 3 joints
├─ Range: [-1.0, 1.0] (normalized)
└─ Maps to actual joint limits in config
```

---

## ⚙️ Configuration Files

### `config/robot_spec.toml` - Robot Parameters
```toml
[body]
mass = 8.5                        # kg
dimensions = [0.6, 0.3, 0.2]      # L×W×H (meters)

[actuators]
continuous_torque = 9.0           # N⋅m
peak_torque = 18.0                # N⋅m
max_velocity = 38.2               # rad/s

[actuators.limits]
hip_x = [-0.8, 0.8]               # radians
hip_y = [-1.2, 1.2]
knee = [-2.4, -0.5]
```

### `config/training.toml` - Hyperparameters
```toml
[training]
n_episodes = 10000
rollout_steps = 2048

[ppo]
learning_rate = 0.0003
gamma = 0.99                      # Discount factor
gae_lambda = 0.95                 # GAE parameter
clip_epsilon = 0.2                # PPO clip range
entropy_coef = 0.01               # Exploration bonus
n_epochs = 10                     # Optimization epochs
batch_size = 64
```

---

## 🎯 Reward Function Weights

```rust
// In gym_env.rs::compute_reward()
forward_velocity:    1.0    // Primary objective
distance_to_goal:   -0.1    // Penalty for being far
stability:           0.5    // Stay upright
energy:             -0.05   // Minimize power usage
action_smoothness:  -0.1    // Avoid jittery movements
goal_reached:      100.0    // Sparse bonus
fallen:           -100.0    // Terminal penalty
```

---

## 📊 Expected Training Metrics

### Episode 0-100 (Exploration)
```
Avg Reward:  -10 to -5
Success Rate: 0%
Behavior: Random flailing, frequent falls
```

### Episode 100-500 (Learning)
```
Avg Reward:  -5 to +2
Success Rate: 5-20%
Behavior: Coordinated steps, occasional walking
```

### Episode 500-1000 (Refinement)
```
Avg Reward:  +2 to +10
Success Rate: 30-60%
Behavior: Consistent walking, reaches goals
```

---

## 🔧 Common Modifications

### Change Robot Size
```toml
# config/robot_spec.toml
[body]
dimensions = [0.8, 0.4, 0.25]  # 33% larger
mass = 15.0                     # Scale mass accordingly
```

### Adjust Learning Rate
```toml
# config/training.toml
[ppo]
learning_rate = 0.001  # 3x faster (riskier)
# or
learning_rate = 0.0001 # 3x slower (safer)
```

### Modify Reward Function
```rust
// crates/environment/src/gym_env.rs
fn compute_reward(&self, action: &[f32]) -> (f32, StepInfo) {
    // Change weights here:
    let r_velocity = ...;
    reward += 2.0 * r_velocity;  // Double velocity importance
    
    // Add new rewards:
    let r_symmetry = -abs(left_legs_force - right_legs_force);
    reward += 0.3 * r_symmetry;
}
```

### Add New Environment Object
```toml
# config/objects.toml
[[objects]]
id = "boulder"
mass = 100.0
dimensions = [1.0, 1.0, 1.0]
shape = "box"
friction = 0.8
restitution = 0.2
static_obj = false
```

---

## 🐛 Debugging Commands

### Enable Physics Logging
```rust
// In main.rs, after env creation:
println!("Physics dt: {}", env.physics.dt);
println!("Gravity: {:?}", env.physics.gravity);

// Inside training loop:
if episode % 10 == 0 {
    if let Some(body) = env.physics.get_body(env.robot.base_body) {
        println!("Robot pos: {:?}", body.translation());
        println!("Robot vel: {:?}", body.linvel());
    }
}
```

### Check Observation Validity
```rust
// After get_observation():
for (i, &v) in obs.iter().enumerate() {
    if !v.is_finite() {
        eprintln!("Invalid obs[{}] = {}", i, v);
    }
}
```

### Monitor Memory Usage
```bash
# While training:
watch -n 1 'ps aux | grep quadruped_rl'

# Check allocations:
cargo build --release
valgrind --leak-check=full ./target/release/quadruped_rl
```

---

## 📈 Performance Tuning

### Speed Up Training
```toml
# Reduce rollout size (faster iterations)
rollout_steps = 128

# Fewer optimization epochs
n_epochs = 5

# Smaller batch size
batch_size = 32
```

### Improve Quality
```toml
# Larger network
hidden_sizes = [512, 256, 128]

# More rollout steps
rollout_steps = 4096

# More optimization
n_epochs = 20
```

### Reduce Memory
```toml
# Smaller rollout buffer
rollout_steps = 64

# Smaller network
hidden_sizes = [128, 64]
```

---

## 🎨 Visualization (Optional)

### Enable Bevy Renderer
```rust
// In Cargo.toml:
[features]
visualization = ["dep:visualization"]

// Run with viz:
cargo run --release --features visualization
```

### Custom Camera Angle
```rust
// In renderer.rs::setup_scene():
commands.spawn(Camera3dBundle {
    transform: Transform::from_xyz(5.0, 5.0, 5.0)  // Farther back
        .looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
    ..default()
});
```

---

## 🔬 Physics Parameters

### Simulation Timestep
```rust
// In gym_env.rs:
PhysicsEngine::new(
    0.002,  // 2ms = 500 Hz (default)
    // Smaller = more accurate but slower
    // Larger = faster but less stable
    ...
)
```

### Gravity
```rust
na::Vector3::new(0.0, -9.81, 0.0)  // Earth (default)
na::Vector3::new(0.0, -3.71, 0.0)  // Mars
na::Vector3::new(0.0, -1.62, 0.0)  // Moon
```

### Ground Friction
```rust
// In world.rs::new():
let ground_collider = ColliderBuilder::cuboid(50.0, 0.1, 50.0)
    .friction(0.8)     // Standard (default)
    // .friction(0.3)  // Ice
    // .friction(1.2)  // Rubber
    .build();
```

---

## 🧪 Testing Checklist

### Smoke Test
```bash
# Should complete without panic:
cargo test --all
cargo run --release -- --episodes 10
```

### Stability Test
```bash
# Run for extended time:
timeout 1h cargo run --release
# Should not crash or OOM
```

### Performance Test
```bash
# Measure throughput:
time cargo run --release -- --episodes 100
# Compare against benchmarks
```

---

## 📦 Dependencies Versions

```toml
rapier3d = "0.17"
nalgebra = "0.32"
ndarray = "0.15"
rand = "0.8"
serde = "1.0"
bevy = "0.12"
rayon = "1.8"
```

**Update all dependencies:**
```bash
cargo update
cargo build --release
```

---

## 💾 Checkpointing

### Save Network Weights
```rust
// TODO: Implement in main.rs
if episode % 100 == 0 {
    let params = trainer.network.get_params();
    let path = format!("checkpoints/episode_{}.bin", episode);
    std::fs::write(path, bytemuck::cast_slice(&params))?;
}
```

### Load Checkpoint
```rust
// TODO: Implement
let bytes = std::fs::read("checkpoints/episode_1000.bin")?;
let params: Vec<f32> = bytemuck::cast_slice(&bytes).to_vec();
trainer.network.set_params(&params);
```

---

## 🌐 Parallel Training (Future)

### Multi-Environment Setup
```rust
use rayon::prelude::*;

let envs: Vec<RobotEnv> = (0..8)
    .map(|_| RobotEnv::new(config.clone(), vec![]))
    .collect();

let rollouts: Vec<_> = envs.par_iter_mut()
    .map(|env| collect_rollout(env, 256))
    .collect();
```

---

## 📊 Logging to File

### Save Training Metrics
```rust
use std::fs::OpenOptions;
use std::io::Write;

let mut log_file = OpenOptions::new()
    .create(true)
    .append(true)
    .open("training.log")?;

writeln!(log_file, "{},{},{}", episode, reward, loss)?;
```

### Plot Results
```bash
# Install gnuplot:
sudo apt install gnuplot

# Plot rewards:
gnuplot -e "set datafile separator ','; 
            plot 'training.log' using 1:2 with lines; 
            pause -1"
```

---

## 🎓 Key Equations

### Policy Gradient
```
∇θ J(θ) ≈ E[ ∇θ log π(a|s) * A(s,a) ]
```

### PPO Clipped Objective
```
L(θ) = E[ min(r(θ)*A, clip(r(θ), 1-ε, 1+ε)*A) ]
where r(θ) = π(a|s) / π_old(a|s)
```

### GAE (Generalized Advantage Estimation)
```
A_t = Σ(γλ)^k * δ_{t+k}
where δ_t = r_t + γV(s_{t+1}) - V(s_t)
```

---

## 🔗 File Relationships

```
main.rs
 ├─→ environment::RobotEnv
 │    ├─→ robot::QuadrupedRobot
 │    │    ├─→ actuator::Actuator
 │    │    └─→ sensor::IMU, Encoder
 │    ├─→ world::World
 │    └─→ physics::PhysicsEngine
 └─→ rl::PPOTrainer
      ├─→ network::ActorCritic
      └─→ replay::RolloutBuffer
```

---

## 🆘 Emergency Fixes

### "Simulation explodes"
```rust
// Reduce dt immediately:
let physics = PhysicsEngine::new(0.0005, gravity);  // 5x smaller

// Or add damping:
for leg in &mut robot.legs {
    leg.hip_x.spec.damping = 2.0;  // 4x more damping
}
```

### "NaN in loss"
```rust
// Add gradient clipping:
for param in &mut params {
    *param = param.clamp(-10.0, 10.0);
}
```

### "Training stuck"
```toml
# Increase exploration:
entropy_coef = 0.05  # 5x more exploration

# Or reset learning rate:
learning_rate = 0.001
```

---

**Quick Ref Version**: 1.0  
**For Full Docs**: See README.md  
**Emergency Support**: Check BUILD_AND_RUN_GUIDE.md
