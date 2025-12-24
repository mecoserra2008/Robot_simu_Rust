# Enhanced Quadruped RL Simulation - Advanced Features

This document describes the significantly enhanced features added to the quadruped robotics reinforcement learning simulation.

## 🎮 Real-Time 3D Visualization

### Features
- **Bevy-based 3D renderer** with real-time physics visualization
- **Interactive camera controls**: Right-click drag to rotate, mouse wheel to zoom
- **Visual debugging**: Physics debug rendering shows colliders and forces
- **Performance metrics UI**: Live display of training statistics
- **Keyboard controls**:
  - `Space`: Pause/Resume simulation
  - `Arrow Up`: Increase simulation speed
  - `Arrow Down`: Decrease simulation speed

### Implementation
```rust
// Located in: crates/visualization/
- renderer.rs: Scene setup, lighting, visual updates
- ui.rs: Training metrics display
- camera.rs: Orbital camera controller
```

## 🤖 Enhanced Robot Capabilities

### Head Movement System
The robot now has a fully articulated head with 2 degrees of freedom:

- **Pitch (up/down)**: ±57 degrees range
- **Yaw (left/right)**: ±86 degrees range
- **Independent actuation**: Separate motors with PD control
- **Sensory advantage**: Can look at objects before interacting

```rust
pub struct Head {
    pub pitch: Actuator,  // Vertical rotation
    pub yaw: Actuator,    // Horizontal rotation
    pub body: BodyHandle,
}
```

### Gripper/Manipulation System
Each of the 4 legs now includes a gripper for object manipulation:

- **Individual control**: 4 independent grippers (1 per leg)
- **Force sensing**: Detects grip force and contact
- **Object tracking**: Knows which object is grasped
- **Smart grasping**: Automatically detects graspable objects

```rust
pub struct Gripper {
    pub open_close: Actuator,      // 0.0 = open, 0.8 = closed
    pub is_grasping: bool,
    pub grasped_object: Option<usize>,
    pub grip_force: f32,
}
```

### Omnidirectional Movement
- Full 360° movement capability
- Coordinated leg control for strafing
- Dynamic balance during complex maneuvers

### Health & Damage System
```rust
pub struct QuadrupedRobot {
    // ... other fields
    pub health: f32,          // Current health (0-100)
    pub max_health: f32,      // Maximum health capacity
}
```

- **Health tracking**: Robot starts with 100 HP
- **Damage sources**:
  - Falling objects (proportional to mass × velocity)
  - Collisions with dangerous objects
  - Falls from heights
  - Excessive impacts
- **Death condition**: Health ≤ 0 ends episode

## 🎯 Multi-Goal Priority System

### Goal Types

The robot can pursue multiple simultaneous goals with dynamic prioritization:

1. **Survive** (Priority: 100 × urgency)
   - Activated when health drops below 30%
   - Time constraint: 1 second
   - Reward: 1000 points

2. **AvoidThreat** (Priority: 80 × urgency)
   - Triggered by falling/fast-moving objects
   - Time constraint: 2 seconds
   - Reward: 500 points

3. **RestoreHealth** (Priority: 70 × urgency)
   - Activated at 30-70% health
   - Time constraint: 5 seconds
   - Reward: 300 points

4. **Navigate** (Priority: 50 × urgency)
   - Move to target location
   - No time constraint
   - Reward: 100 points

5. **GrabObject** (Priority: 40 × priority)
   - Pick up specified object
   - Reward: 150 points

6. **PlaceObject** (Priority: 35 × priority)
   - Move object to location
   - Reward: 200 points

7. **Explore** (Priority: 20 × curiosity)
   - Discover new areas
   - Reward: 50 points

### Dynamic Prioritization

Goals are automatically reprioritized based on:
- **Time urgency**: Priority increases as deadline approaches
- **Current threats**: Dangerous objects elevate avoidance goals
- **Health status**: Low health triggers survival/healing goals
- **Expected rewards**: Higher reward goals get preference
- **Completion status**: Completed goals are removed

```rust
impl Goal {
    pub fn get_effective_priority(&self, current_time: f32) -> f32 {
        let base_priority = self.goal_type.get_priority();

        // Increase priority as deadline approaches
        if let Some(deadline) = self.deadline {
            let urgency_multiplier = calculate_urgency(current_time, deadline);
            base_priority * urgency_multiplier
        } else {
            base_priority
        }
    }
}
```

## 🎨 Diverse Realistic Objects

### Object Shapes
- **Box**: Crates, containers, blocks
- **Sphere**: Balls (various materials)
- **Cylinder**: Pillars, cans
- **Capsule**: Logs, bottles
- **Cone**: Sharp objects (dangerous!)

### Material Types

Each material has realistic physics properties:

| Material | Density (kg/m³) | Friction | Restitution | Health Mult | Characteristics |
|----------|----------------|----------|-------------|-------------|-----------------|
| **Wood** | 600 | 0.6 | 0.3 | 0.8 | Light, moderate durability |
| **Metal** | 7800 | 0.7 | 0.5 | 2.0 | Very heavy, high durability, dangerous when falling |
| **Glass** | 2500 | 0.4 | 0.1 | 0.3 | Fragile, shatters easily |
| **Rubber** | 1100 | 1.2 | 0.9 | 1.0 | Bouncy, high friction |
| **Plastic** | 950 | 0.5 | 0.4 | 0.7 | Light, moderate properties |
| **Stone** | 2600 | 0.8 | 0.2 | 1.5 | Heavy, very durable |

### Object Examples

```toml
# Dangerous heavy metal sphere
[[objects]]
id = "metal_sphere_heavy"
shape = { Sphere = { radius = 0.25 } }
material = "Metal"
static_obj = false  # Can fall and cause damage!

# Fragile glass vase
[[objects]]
id = "glass_vase"
shape = { Cylinder = { height = 0.5, radius = 0.15 } }
material = "Glass"
static_obj = false  # Breaks on impact

# Bouncy rubber ball
[[objects]]
id = "rubber_ball_bouncy"
shape = { Sphere = { radius = 0.15 } }
material = "Rubber"
static_obj = false  # High restitution
```

## ⚠️ Threat Detection & Response

### Automatic Threat Detection

The system continuously monitors for dangers:

```rust
fn detect_and_respond_to_threats(&mut self) {
    for obj in &self.objects {
        if obj.is_dangerous {  // Falling fast
            let distance = calculate_distance(robot, obj);
            let urgency = (3.0 - distance) / 3.0;

            if distance < 3.0 {
                self.goal_manager.add_goal(GoalType::AvoidThreat {
                    threat_pos: obj_pos,
                    threat_velocity: obj_vel,
                    urgency,
                });
            }
        }
    }
}
```

### Danger Criteria

Objects are marked as dangerous when:
- **Velocity > 5 m/s** AND
- **Falling** (vertical velocity < -2 m/s)
- **Damage potential** = (speed × mass) / 100, max 50 HP

### Collision Detection

```rust
fn check_collisions(&mut self) {
    for obj in &self.objects {
        if obj.is_dangerous {
            let distance = calculate_distance(robot, obj);
            if distance < 0.5 {  // Collision!
                self.robot.health -= obj.damage_on_contact;
            }
        }
    }
}
```

## 📊 Enhanced Observation Space

Total dimensions: **81** (increased from 51)

| Component | Dimensions | Description |
|-----------|-----------|-------------|
| Base state | 13 | Position, rotation, linear velocity, angular velocity |
| Joint state | 24 | 12 joints × (position, velocity) |
| Contact sensors | 4 | Foot contact detection |
| Head state | 4 | Pitch/yaw position & velocity |
| Gripper state | 4 | 4 grippers × open/close state |
| Previous actions | 18 | All previous joint commands |
| Health | 1 | Normalized health ratio |
| Current goals | 12 | Top 3 goals × 4 values each |
| Threats | 2 | Nearest threat distance & velocity |

## 🎮 Enhanced Action Space

Total dimensions: **18** (increased from 12)

| Component | Dimensions | Range | Description |
|-----------|-----------|-------|-------------|
| Leg joints | 12 | [-1, 1] | 4 legs × 3 joints each |
| Head pitch | 1 | [-1, 1] | Up/down rotation |
| Head yaw | 1 | [-1, 1] | Left/right rotation |
| Grippers | 4 | [0, 1] | 4 independent grippers |

## 🏆 Advanced Reward Function

```rust
reward =
    + goal_based_reward()          // Dynamic based on active goal
    + health_ratio * 5.0           // Survival incentive
    + stability_reward * 0.5       // Stay upright
    + (-energy_usage) * 0.05       // Efficiency
    + (-action_smoothness) * 0.1   // Smooth movements
    + threat_avoidance_bonus       // Dodging dangerous objects
    + object_manipulation_reward   // Successful grasping
    + 100.0 (if goal completed)
    - 100.0 (if fallen)
    - 200.0 (if destroyed)
```

### Goal-Specific Rewards

- **Navigate**: Velocity toward target + distance reduction
- **AvoidThreat**: Increases with distance from threat
- **GrabObject**: Bonus for successful grasp
- **Survive**: Proportional to health maintained

## 🔧 Configuration

### Training Configuration (`config/training_advanced.toml`)

```toml
[training]
n_episodes = 50000
rollout_steps = 4096
use_visualization = true

[ppo]
learning_rate = 0.0003
entropy_coef = 0.02       # Increased exploration
n_epochs = 15             # More training per rollout
batch_size = 128

[network]
hidden_sizes = [512, 256, 128]  # Larger network for complexity

[environment]
max_steps = 2000
enable_damage = true
enable_multi_goals = true

[reward_weights]
velocity = 1.0
health = 5.0
goal_completion = 100.0
threat_avoidance = 50.0
death_penalty = -200.0
```

## 🚀 Usage Examples

### Basic Training with Visualization

```bash
RUST_LOG=info cargo run --release --features visualization
```

### Advanced Training

```rust
use environment::AdvancedRobotEnv;
use robot::RobotConfig;

let robot_config = load_robot_config("config/robot_spec.toml")?;
let mut env = AdvancedRobotEnv::new(robot_config);

for episode in 0..10000 {
    let mut obs = env.reset();
    let mut done = false;

    while !done {
        let action = policy.sample_action(&obs);
        let (next_obs, reward, done, info) = env.step(&action);

        // Training logic...
        obs = next_obs;
    }
}
```

## 📈 Expected Performance

### Training Progression

| Episodes | Avg Reward | Success Rate | Behavior |
|----------|-----------|--------------|----------|
| 0-500 | -20 to -5 | 0-5% | Learning to stand, avoid falls |
| 500-2000 | -5 to +5 | 5-20% | Basic walking, some goal completion |
| 2000-10000 | +5 to +15 | 20-50% | Strategic movement, threat avoidance |
| 10000+ | +15 to +30 | 50-80% | Multi-goal coordination, object manipulation |

### Key Milestones

- **Episode ~100**: Learns to stand without falling
- **Episode ~500**: Basic forward locomotion
- **Episode ~1000**: Reaches simple navigation goals
- **Episode ~2000**: Begins avoiding threats
- **Episode ~5000**: Successfully manipulates objects
- **Episode ~10000**: Coordinates multiple goals effectively

## 🔬 Advanced Features Details

### Multi-Temporal Goal Management

Goals operate on different timescales:
- **Immediate** (< 1s): Survival, dodge falling objects
- **Short-term** (1-5s): Restore health, avoid threats
- **Medium-term** (5-30s): Navigate, grab objects
- **Long-term** (no limit): Explore, complete complex tasks

### Adaptive Behavior

The robot learns to:
1. **Prioritize survival** when health is low
2. **Dodge threats** while maintaining objectives
3. **Use head** to scout before moving
4. **Manipulate objects** with grippers for rewards
5. **Adapt strategy** based on material properties
6. **Coordinate goals** for maximum efficiency

## 🎯 Future Enhancements

Potential additions:
- Multi-robot cooperation
- Tool use and construction
- Complex object puzzles
- Dynamic environment changes
- Adversarial scenarios
- Transfer learning to real hardware

---

**Version**: 2.0
**Last Updated**: 2025-12-24
**Minimum Rust Version**: 1.70+
**Dependencies**: Rapier3D 0.17, Bevy 0.12, nalgebra 0.32
